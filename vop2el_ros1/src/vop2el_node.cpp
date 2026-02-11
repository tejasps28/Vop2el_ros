#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <boost/bind/bind.hpp>
#include <stdexcept>
#include <algorithm>
#include <deque>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <utility>
#include <atomic>
#include <cmath>
#include <limits>
#include <string>
#include <cctype>

#include "Vop2elAlgorithm.h"
#include "Common.h"
#include "Utils.h"

namespace
{
geometry_msgs::Pose PoseFromEigen(const Eigen::Affine3d& transform)
{
    geometry_msgs::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();

    Eigen::Quaterniond q(transform.rotation());
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}

geometry_msgs::Transform TransformFromEigen(const Eigen::Affine3d& transform)
{
    geometry_msgs::Transform tf;
    tf.translation.x = transform.translation().x();
    tf.translation.y = transform.translation().y();
    tf.translation.z = transform.translation().z();

    Eigen::Quaterniond q(transform.rotation());
    tf.rotation.x = q.x();
    tf.rotation.y = q.y();
    tf.rotation.z = q.z();
    tf.rotation.w = q.w();
    return tf;
}
}

class Vop2elNode
{
public:
    Vop2elNode() :
        nh_(),
        pnh_("~"),
        it_(nh_),
        tf_buffer_(),
        tf_listener_(tf_buffer_)
    {
        LoadParams();

        left_image_sub_.subscribe(it_, left_image_topic_, queue_size_);
        right_image_sub_.subscribe(it_, right_image_topic_, queue_size_);
        if (use_camera_info_)
        {
            left_info_sub_.subscribe(nh_, left_camera_info_topic_, queue_size_);
            right_info_sub_.subscribe(nh_, right_camera_info_topic_, queue_size_);
            sync_with_info_.reset(new SyncWithInfo(SyncPolicyWithInfo(queue_size_),
                                                   left_image_sub_, right_image_sub_, left_info_sub_, right_info_sub_));
            sync_with_info_->registerCallback(boost::bind(&Vop2elNode::StereoWithInfoCallback, this,
                                                          boost::placeholders::_1, boost::placeholders::_2,
                                                          boost::placeholders::_3, boost::placeholders::_4));
        }
        else
        {
            {
                std::lock_guard<std::mutex> lock(algorithm_mutex_);
                algorithm_.reset(new Vop2el::Vop2elAlgorithm(params_));
                camera_ready_ = true;
            }
            if (sync_policy_ == "exact")
            {
                sync_images_exact_.reset(new SyncImagesExact(SyncPolicyImagesExact(queue_size_), left_image_sub_, right_image_sub_));
                sync_images_exact_->registerCallback(boost::bind(&Vop2elNode::StereoImagesCallback, this,
                                                                 boost::placeholders::_1, boost::placeholders::_2));
            }
            else
            {
                sync_images_approx_.reset(new SyncImagesApprox(SyncPolicyImagesApprox(queue_size_), left_image_sub_, right_image_sub_));
                sync_images_approx_->registerCallback(boost::bind(&Vop2elNode::StereoImagesCallback, this,
                                                                  boost::placeholders::_1, boost::placeholders::_2));
            }
        }

        if (publish_odom_)
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 5, false);
        if (publish_path_)
            path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
        if (publish_features_)
            features_pub_ = nh_.advertise<sensor_msgs::PointCloud>(features_topic_, 5, false);
        if (publish_features_image_)
            features_image_pub_ = it_.advertise(features_image_topic_, 1);
        if (publish_debug_)
            debug_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(debug_topic_, 5, false);

        worker_thread_ = std::thread(&Vop2elNode::ProcessingLoop, this);
    }

    ~Vop2elNode()
    {
        StopWorker();
    }

private:
    using SyncPolicyWithInfo = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                                              sensor_msgs::Image,
                                                                              sensor_msgs::CameraInfo,
                                                                              sensor_msgs::CameraInfo>;
    using SyncWithInfo = message_filters::Synchronizer<SyncPolicyWithInfo>;
    using SyncPolicyImagesApprox = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                                                   sensor_msgs::Image>;
    using SyncImagesApprox = message_filters::Synchronizer<SyncPolicyImagesApprox>;
    using SyncPolicyImagesExact = message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                                           sensor_msgs::Image>;
    using SyncImagesExact = message_filters::Synchronizer<SyncPolicyImagesExact>;
    struct BufferedStereoFrame
    {
        cv::Mat left;
        cv::Mat right;
        ros::Time stamp;
    };
    struct SelectorCandidate
    {
        BufferedStereoFrame frame;
        double score = 0.0;
    };

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;

    image_transport::SubscriberFilter left_image_sub_;
    image_transport::SubscriberFilter right_image_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> right_info_sub_;
    std::unique_ptr<SyncWithInfo> sync_with_info_;
    std::unique_ptr<SyncImagesApprox> sync_images_approx_;
    std::unique_ptr<SyncImagesExact> sync_images_exact_;

    ros::Publisher odom_pub_;
    ros::Publisher path_pub_;
    ros::Publisher features_pub_;
    ros::Publisher debug_pub_;
    image_transport::Publisher features_image_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::unique_ptr<Vop2el::Vop2elAlgorithm> algorithm_;
    Vop2el::Vop2elParameters params_;
    std::mutex algorithm_mutex_;

    bool camera_ready_ = false;

    std::string left_image_topic_;
    std::string right_image_topic_;
    std::string left_camera_info_topic_;
    std::string right_camera_info_topic_;
    std::string odom_topic_;
    std::string path_topic_;
    std::string features_topic_;
    std::string features_image_topic_;
    std::string debug_topic_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string left_frame_override_;
    std::string right_frame_override_;
    std::string ini_file_;

    bool publish_tf_ = true;
    bool publish_odom_ = true;
    bool publish_path_ = true;
    bool publish_features_ = true;
    bool publish_features_image_ = true;
    bool publish_debug_ = true;
    bool use_camera_info_ = true;
    bool use_rectified_ = true;
    double tf_lookup_timeout_ = 0.1;
    int queue_size_ = 10;
    int input_buffer_size_ = 5;
    int process_every_n_ = 1;
    double target_process_rate_hz_ = 0.0;
    std::string sync_policy_ = "exact";
    double max_stereo_dt_sec_ = 0.002;
    bool force_grayscale_ = true;
    std::string selector_mode_ = "stride";
    int selection_buffer_size_ = 3;
    double selection_max_latency_sec_ = 0.15;
    double min_sharpness_ = 0.0;
    double min_brightness_ = -1.0;
    double max_brightness_ = 256.0;
    int path_publish_stride_ = 1;
    bool skip_publish_on_fallback_ = false;
    bool drop_oldest_when_full_ = true;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<BufferedStereoFrame> frame_queue_;
    std::mutex selector_mutex_;
    ros::Time last_selected_stamp_;
    std::deque<SelectorCandidate> selector_candidates_;
    bool worker_running_ = true;
    std::thread worker_thread_;
    std::atomic<uint64_t> frames_received_{0};
    std::atomic<uint64_t> frames_skipped_selector_{0};
    std::atomic<uint64_t> frames_rejected_quality_{0};
    std::atomic<uint64_t> frames_selected_quality_buffer_{0};
    std::atomic<uint64_t> frames_skipped_desync_{0};
    std::atomic<uint64_t> frames_enqueued_{0};
    std::atomic<uint64_t> frames_dropped_{0};
    std::atomic<uint64_t> frames_processed_{0};
    std::atomic<uint64_t> frames_published_{0};
    std::atomic<uint64_t> frames_skipped_publish_fallback_{0};
    ros::WallTime last_process_wall_;
    bool has_last_process_wall_ = false;
    double processing_fps_ = 0.0;
    uint64_t path_publish_counter_ = 0;
    nav_msgs::Path path_msg_;
    std::vector<Eigen::Affine3d> published_poses_;

    void LoadParams()
    {
        pnh_.param("left_image_topic", left_image_topic_, std::string("/stereo/left/image_rect"));
        pnh_.param("right_image_topic", right_image_topic_, std::string("/stereo/right/image_rect"));
        pnh_.param("left_camera_info_topic", left_camera_info_topic_, std::string("/stereo/left/camera_info"));
        pnh_.param("right_camera_info_topic", right_camera_info_topic_, std::string("/stereo/right/camera_info"));
        pnh_.param("odom_topic", odom_topic_, std::string("/vo/odom"));
        pnh_.param("path_topic", path_topic_, std::string("/vo/path"));
        pnh_.param("features_topic", features_topic_, std::string("/vo/features"));
        pnh_.param("features_image_topic", features_image_topic_, std::string("/vo/features_image"));
        pnh_.param("debug_topic", debug_topic_, std::string("/vo/debug"));
        pnh_.param("odom_frame", odom_frame_, std::string("odom"));
        pnh_.param("base_frame", base_frame_, std::string("camera_left"));
        pnh_.param("left_frame_override", left_frame_override_, std::string(""));
        pnh_.param("right_frame_override", right_frame_override_, std::string(""));
        pnh_.param("ini_file", ini_file_, std::string(""));

        pnh_.param("publish_tf", publish_tf_, true);
        pnh_.param("publish_odom", publish_odom_, true);
        pnh_.param("publish_path", publish_path_, true);
        pnh_.param("publish_features", publish_features_, true);
        pnh_.param("publish_features_image", publish_features_image_, true);
        pnh_.param("publish_debug", publish_debug_, true);
        pnh_.param("use_camera_info", use_camera_info_, true);
        pnh_.param("use_rectified", use_rectified_, true);
        pnh_.param("tf_lookup_timeout", tf_lookup_timeout_, 0.1);
        pnh_.param("queue_size", queue_size_, 10);
        pnh_.param("input_buffer_size", input_buffer_size_, 5);
        pnh_.param("process_every_n", process_every_n_, 1);
        pnh_.param("target_process_rate_hz", target_process_rate_hz_, 0.0);
        pnh_.param("sync_policy", sync_policy_, std::string("exact"));
        pnh_.param("max_stereo_dt_sec", max_stereo_dt_sec_, 0.002);
        pnh_.param("force_grayscale", force_grayscale_, true);
        pnh_.param("selector_mode", selector_mode_, std::string("stride"));
        pnh_.param("selection_buffer_size", selection_buffer_size_, 3);
        pnh_.param("selection_max_latency_sec", selection_max_latency_sec_, 0.15);
        pnh_.param("min_sharpness", min_sharpness_, 0.0);
        pnh_.param("min_brightness", min_brightness_, -1.0);
        pnh_.param("max_brightness", max_brightness_, 256.0);
        pnh_.param("path_publish_stride", path_publish_stride_, 1);
        pnh_.param("skip_publish_on_fallback", skip_publish_on_fallback_, false);
        pnh_.param("drop_oldest_when_full", drop_oldest_when_full_, true);
        std::transform(sync_policy_.begin(), sync_policy_.end(), sync_policy_.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (sync_policy_ != "exact" && sync_policy_ != "approximate")
        {
            ROS_WARN("Unknown sync_policy '%s'. Falling back to 'exact'.", sync_policy_.c_str());
            sync_policy_ = "exact";
        }
        process_every_n_ = std::max(1, process_every_n_);
        target_process_rate_hz_ = std::max(0.0, target_process_rate_hz_);
        selection_buffer_size_ = std::max(1, selection_buffer_size_);
        selection_max_latency_sec_ = std::max(0.0, selection_max_latency_sec_);
        max_brightness_ = std::max(max_brightness_, min_brightness_);
        path_publish_stride_ = std::max(1, path_publish_stride_);
        max_stereo_dt_sec_ = std::max(0.0, max_stereo_dt_sec_);
        if (selector_mode_ != "stride" && selector_mode_ != "quality_buffer")
        {
            ROS_WARN("Unknown selector_mode '%s'. Falling back to 'stride'.", selector_mode_.c_str());
            selector_mode_ = "stride";
        }

        if (use_camera_info_)
        {
            ROS_WARN("INI-only wrapper mode active. Forcing use_camera_info=false.");
            use_camera_info_ = false;
        }

        if (ini_file_.empty())
        {
            ROS_FATAL("INI-only wrapper mode requires ini_file. Provide a valid Vop2elParameters file.");
            throw std::runtime_error("Missing ini_file (INI-only mode)");
        }

        try
        {
            Utils::GenerateVop2elParamsFromIniFile(ini_file_, params_);
            ROS_INFO("Loaded Vop2el parameters from ini_file: %s", ini_file_.c_str());
            ROS_INFO("INI-only mode: YAML algorithm parameter blocks are ignored.");
        }
        catch (const std::exception& ex)
        {
            ROS_FATAL("Failed to load ini_file '%s': %s", ini_file_.c_str(), ex.what());
            throw;
        }

        const bool left_looks_color = left_image_topic_.find("camera_color") != std::string::npos;
        const bool right_looks_color = right_image_topic_.find("camera_color") != std::string::npos;
        if (left_looks_color || right_looks_color)
        {
            ROS_WARN("INI-only mode with color topics: ensure ini_file calibration belongs to the same color cameras.");
        }

        ROS_INFO("Frame selection: mode=%s, process_every_n=%d, target_process_rate_hz=%.3f, "
                 "selection_buffer_size=%d, selection_max_latency_sec=%.3f, min_sharpness=%.3f, sync_policy=%s, max_stereo_dt_sec=%.4f, "
                 "brightness=[%.3f, %.3f], skip_publish_on_fallback=%s",
                 selector_mode_.c_str(), process_every_n_, target_process_rate_hz_,
                 selection_buffer_size_, selection_max_latency_sec_, min_sharpness_, sync_policy_.c_str(), max_stereo_dt_sec_,
                 min_brightness_, max_brightness_, skip_publish_on_fallback_ ? "true" : "false");
    }

    bool UpdateCameraParams(const sensor_msgs::CameraInfoConstPtr& left_info,
                            const sensor_msgs::CameraInfoConstPtr& right_info)
    {
        if (left_info->K[0] == 0.0 || left_info->K[4] == 0.0)
        {
            ROS_WARN_THROTTLE(2.0, "Invalid left CameraInfo K matrix.");
            return false;
        }

        cv::Mat calibration(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                calibration.at<double>(r, c) = left_info->K[r * 3 + c];

        params_.CameraParams.CalibrationMatrix = calibration.clone();
        params_.CameraParams.cols = static_cast<int>(left_info->width);
        params_.CameraParams.rows = static_cast<int>(left_info->height);

        if (use_rectified_)
        {
            if (right_info->P[0] == 0.0 || right_info->P[5] == 0.0)
            {
                ROS_WARN_THROTTLE(2.0, "Right CameraInfo P matrix is invalid for rectified mode.");
                return false;
            }

            cv::Mat extrinsic_rotation = cv::Mat::eye(3, 3, CV_64F);
            cv::Mat extrinsic_translation = cv::Mat::zeros(3, 1, CV_64F);
            extrinsic_translation.at<double>(0) = -right_info->P[3] / right_info->P[0];
            extrinsic_translation.at<double>(1) = -right_info->P[7] / right_info->P[5];
            extrinsic_translation.at<double>(2) = (right_info->P[10] != 0.0) ? -right_info->P[11] / right_info->P[10] : 0.0;

            params_.CameraParams.ExtrinsicRotation = extrinsic_rotation;
            params_.CameraParams.ExtrinsicTranslation = extrinsic_translation;
            return true;
        }

        std::string left_frame = left_frame_override_.empty() ? left_info->header.frame_id : left_frame_override_;
        std::string right_frame = right_frame_override_.empty() ? right_info->header.frame_id : right_frame_override_;

        if (left_frame.empty() || right_frame.empty())
        {
            ROS_WARN_THROTTLE(2.0, "CameraInfo frame_id is empty; set left_frame_override/right_frame_override.");
            return false;
        }

        geometry_msgs::TransformStamped tf_left_to_right;
        try
        {
            tf_left_to_right = tf_buffer_.lookupTransform(right_frame, left_frame, ros::Time(0), ros::Duration(tf_lookup_timeout_));
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_THROTTLE(2.0, "TF lookup failed: %s", ex.what());
            return false;
        }

        tf2::Quaternion q;
        tf2::fromMsg(tf_left_to_right.transform.rotation, q);
        tf2::Matrix3x3 rot(q);

        cv::Mat extrinsic_rotation(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                extrinsic_rotation.at<double>(r, c) = rot[r][c];

        cv::Mat extrinsic_translation(3, 1, CV_64F);
        extrinsic_translation.at<double>(0) = tf_left_to_right.transform.translation.x;
        extrinsic_translation.at<double>(1) = tf_left_to_right.transform.translation.y;
        extrinsic_translation.at<double>(2) = tf_left_to_right.transform.translation.z;

        params_.CameraParams.ExtrinsicRotation = extrinsic_rotation;
        params_.CameraParams.ExtrinsicTranslation = extrinsic_translation;
        return true;
    }

    bool ConvertImageMsg(const sensor_msgs::ImageConstPtr& msg, cv::Mat& out_image)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            if (force_grayscale_)
                cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
            else
                cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        }
        catch (const cv_bridge::Exception& ex)
        {
            ROS_WARN_THROTTLE(2.0, "cv_bridge exception: %s", ex.what());
            return false;
        }

        out_image = cv_ptr->image.clone();
        if (!out_image.isContinuous())
            out_image = out_image.clone();
        return true;
    }

    bool ShouldSelectFrame(const ros::Time& stamp, uint64_t received)
    {
        if (process_every_n_ > 1 && ((received - 1) % static_cast<uint64_t>(process_every_n_)) != 0)
        {
            frames_skipped_selector_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        if (target_process_rate_hz_ > 0.0)
        {
            const double min_dt = 1.0 / target_process_rate_hz_;
            std::lock_guard<std::mutex> lock(selector_mutex_);
            if (!last_selected_stamp_.isZero() && stamp.isValid() && stamp >= last_selected_stamp_)
            {
                const double dt = (stamp - last_selected_stamp_).toSec();
                if (dt < min_dt)
                {
                    frames_skipped_selector_.fetch_add(1, std::memory_order_relaxed);
                    return false;
                }
            }
            if (stamp.isValid())
                last_selected_stamp_ = stamp;
        }

        return true;
    }

    bool ComputeQualityMetrics(const cv::Mat& image, double& sharpness, double& brightness) const
    {
        if (image.empty())
            return false;

        cv::Mat gray;
        if (image.channels() == 1)
            gray = image;
        else if (image.channels() == 3)
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        else if (image.channels() == 4)
            cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
        else
            return false;

        cv::Scalar mean_val = cv::mean(gray);
        brightness = mean_val[0];

        cv::Mat lap;
        cv::Laplacian(gray, lap, CV_64F);
        cv::Scalar mu, sigma;
        cv::meanStdDev(lap, mu, sigma);
        sharpness = sigma[0] * sigma[0];
        return true;
    }

    bool PassQualityGate(const cv::Mat& left_img, double& score, double& brightness)
    {
        if (!ComputeQualityMetrics(left_img, score, brightness))
            return false;

        if (min_sharpness_ > 0.0 && score < min_sharpness_)
            return false;
        if (min_brightness_ >= 0.0 && brightness < min_brightness_)
            return false;
        if (max_brightness_ <= 255.0 && brightness > max_brightness_)
            return false;
        return true;
    }

    void PushFrameToQueue(BufferedStereoFrame&& frame)
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        while (static_cast<int>(frame_queue_.size()) >= std::max(1, input_buffer_size_))
        {
            if (!drop_oldest_when_full_)
            {
                ROS_WARN_THROTTLE(2.0, "Input buffer full; dropping newest stereo frame.");
                frames_dropped_.fetch_add(1, std::memory_order_relaxed);
                return;
            }
            frame_queue_.pop_front();
            frames_dropped_.fetch_add(1, std::memory_order_relaxed);
            ROS_WARN_THROTTLE(2.0, "Input buffer full; dropping oldest stereo frame.");
        }

        frame_queue_.push_back(std::move(frame));
        frames_enqueued_.fetch_add(1, std::memory_order_relaxed);
        lock.unlock();
        queue_cv_.notify_one();
    }

    bool SelectFromQualityBuffer(const BufferedStereoFrame& frame, double score, BufferedStereoFrame& selected)
    {
        std::lock_guard<std::mutex> lock(selector_mutex_);
        selector_candidates_.push_back(SelectorCandidate{frame, score});
        if (static_cast<int>(selector_candidates_.size()) > selection_buffer_size_)
            selector_candidates_.pop_front();

        bool flush_due_to_size = static_cast<int>(selector_candidates_.size()) >= selection_buffer_size_;
        bool flush_due_to_latency = false;
        if (!selector_candidates_.empty() && selection_max_latency_sec_ > 0.0 && frame.stamp.isValid() && selector_candidates_.front().frame.stamp.isValid())
        {
            const double age = (frame.stamp - selector_candidates_.front().frame.stamp).toSec();
            flush_due_to_latency = age >= selection_max_latency_sec_;
        }
        if (!flush_due_to_size && !flush_due_to_latency)
            return false;

        auto best_it = std::max_element(selector_candidates_.begin(), selector_candidates_.end(),
                                        [](const SelectorCandidate& a, const SelectorCandidate& b)
                                        {
                                            return a.score < b.score;
                                        });
        if (best_it == selector_candidates_.end())
            return false;

        selected = best_it->frame;
        selector_candidates_.clear();
        frames_selected_quality_buffer_.fetch_add(1, std::memory_order_relaxed);

        if (target_process_rate_hz_ > 0.0)
        {
            const double min_dt = 1.0 / target_process_rate_hz_;
            if (!last_selected_stamp_.isZero() && selected.stamp.isValid() && selected.stamp >= last_selected_stamp_)
            {
                const double dt = (selected.stamp - last_selected_stamp_).toSec();
                if (dt < min_dt)
                {
                    frames_skipped_selector_.fetch_add(1, std::memory_order_relaxed);
                    return false;
                }
            }
        }
        if (selected.stamp.isValid())
            last_selected_stamp_ = selected.stamp;
        return true;
    }

    void EnqueueStereoPair(const sensor_msgs::ImageConstPtr& left_msg,
                           const sensor_msgs::ImageConstPtr& right_msg)
    {
        if (left_msg->header.stamp.isValid() && right_msg->header.stamp.isValid())
        {
            const double dt = std::abs((left_msg->header.stamp - right_msg->header.stamp).toSec());
            if (dt > max_stereo_dt_sec_)
            {
                frames_skipped_desync_.fetch_add(1, std::memory_order_relaxed);
                ROS_WARN_THROTTLE(2.0, "Skipping desynchronized stereo pair. |dt|=%.6f sec (> %.6f sec)", dt, max_stereo_dt_sec_);
                return;
            }
        }

        const uint64_t received = frames_received_.fetch_add(1, std::memory_order_relaxed) + 1;
        {
            std::lock_guard<std::mutex> lock(algorithm_mutex_);
            if (!camera_ready_ || !algorithm_)
                return;
        }

        cv::Mat left_img;
        cv::Mat right_img;
        if (!ConvertImageMsg(left_msg, left_img) || !ConvertImageMsg(right_msg, right_img))
            return;
        BufferedStereoFrame frame{left_img, right_img, left_msg->header.stamp};

        if (selector_mode_ == "quality_buffer")
        {
            double sharpness = 0.0;
            double brightness = 0.0;
            if (!PassQualityGate(left_img, sharpness, brightness))
            {
                frames_rejected_quality_.fetch_add(1, std::memory_order_relaxed);
                return;
            }

            BufferedStereoFrame selected;
            if (!SelectFromQualityBuffer(frame, sharpness, selected))
                return;
            PushFrameToQueue(std::move(selected));
            return;
        }

        if (!ShouldSelectFrame(left_msg->header.stamp, received))
            return;
        PushFrameToQueue(std::move(frame));
    }

    void PublishFeatures(const std::vector<cv::Point2f>& features, const ros::Time& stamp)
    {
        if (!publish_features_ || !features_pub_)
            return;

        sensor_msgs::PointCloud cloud;
        cloud.header.stamp = stamp;
        cloud.header.frame_id = base_frame_;
        cloud.points.reserve(features.size());
        for (const auto& kp : features)
        {
            geometry_msgs::Point32 p;
            p.x = kp.x;
            p.y = kp.y;
            p.z = 0.0f;
            cloud.points.push_back(p);
        }
        features_pub_.publish(cloud);
    }

    void PublishFeaturesImage(const cv::Mat& left_image,
                              const std::vector<cv::Point2f>& features,
                              const ros::Time& stamp)
    {
        if (!publish_features_image_ || !features_image_pub_)
            return;
        if (left_image.empty())
            return;

        cv::Mat base_gray;
        if (left_image.channels() == 1)
            base_gray = left_image;
        else if (left_image.channels() == 3)
            cv::cvtColor(left_image, base_gray, cv::COLOR_BGR2GRAY);
        else if (left_image.channels() == 4)
            cv::cvtColor(left_image, base_gray, cv::COLOR_BGRA2GRAY);
        else
            return;

        cv::Mat overlay_bgr;
        cv::cvtColor(base_gray, overlay_bgr, cv::COLOR_GRAY2BGR);
        for (const auto& kp : features)
            cv::circle(overlay_bgr, kp, 2, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);

        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = stamp;
        out_msg.header.frame_id = base_frame_;
        out_msg.encoding = "bgr8";
        out_msg.image = overlay_bgr;
        features_image_pub_.publish(out_msg.toImageMsg());
    }

    void PublishDebug(const ros::Time& stamp,
                      size_t feature_count,
                      size_t pose_count,
                      int match_count,
                      int inlier_count,
                      bool fallback_used,
                      bool extrapolated_on_failure,
                      int failure_reason)
    {
        if (!publish_debug_ || !debug_pub_)
            return;

        size_t queue_depth = 0;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            queue_depth = frame_queue_.size();
        }
        diagnostic_msgs::DiagnosticArray diag;
        diag.header.stamp = stamp;

        diagnostic_msgs::DiagnosticStatus status;
        status.name = "vop2el_ros1";
        status.hardware_id = "vop2el_node";
        status.level = diagnostic_msgs::DiagnosticStatus::OK;
        status.message = "running";

        auto add_key_value = [&status](const std::string& key, const std::string& value)
        {
            diagnostic_msgs::KeyValue kv;
            kv.key = key;
            kv.value = value;
            status.values.push_back(kv);
        };

        add_key_value("queue_depth", std::to_string(queue_depth));
        add_key_value("input_buffer_size", std::to_string(input_buffer_size_));
        add_key_value("selector_mode", selector_mode_);
        add_key_value("process_every_n", std::to_string(process_every_n_));
        add_key_value("target_process_rate_hz", std::to_string(target_process_rate_hz_));
        add_key_value("frames_received", std::to_string(frames_received_.load(std::memory_order_relaxed)));
        add_key_value("frames_skipped_selector", std::to_string(frames_skipped_selector_.load(std::memory_order_relaxed)));
        add_key_value("frames_skipped_desync", std::to_string(frames_skipped_desync_.load(std::memory_order_relaxed)));
        add_key_value("frames_rejected_quality", std::to_string(frames_rejected_quality_.load(std::memory_order_relaxed)));
        add_key_value("frames_selected_quality_buffer", std::to_string(frames_selected_quality_buffer_.load(std::memory_order_relaxed)));
        add_key_value("frames_enqueued", std::to_string(frames_enqueued_.load(std::memory_order_relaxed)));
        add_key_value("frames_processed", std::to_string(frames_processed_.load(std::memory_order_relaxed)));
        add_key_value("frames_published", std::to_string(frames_published_.load(std::memory_order_relaxed)));
        add_key_value("frames_skipped_publish_fallback", std::to_string(frames_skipped_publish_fallback_.load(std::memory_order_relaxed)));
        add_key_value("frames_dropped", std::to_string(frames_dropped_.load(std::memory_order_relaxed)));
        add_key_value("processing_fps", std::to_string(processing_fps_));
        add_key_value("feature_count", std::to_string(feature_count));
        add_key_value("match_count", std::to_string(match_count));
        add_key_value("inlier_count", std::to_string(inlier_count));
        add_key_value("fallback_used", fallback_used ? "true" : "false");
        add_key_value("skip_publish_on_fallback", skip_publish_on_fallback_ ? "true" : "false");
        add_key_value("extrapolated_on_failure", extrapolated_on_failure ? "true" : "false");
        add_key_value("failure_reason", std::to_string(failure_reason));
        add_key_value("extrapolate_on_failure", params_.ExtrapolateOnFailure ? "true" : "false");
        add_key_value("pose_count", std::to_string(pose_count));
        add_key_value("use_camera_info", use_camera_info_ ? "true" : "false");
        add_key_value("use_rectified", use_rectified_ ? "true" : "false");
        add_key_value("sync_policy", sync_policy_);
        add_key_value("max_stereo_dt_sec", std::to_string(max_stereo_dt_sec_));
        add_key_value("force_grayscale", force_grayscale_ ? "true" : "false");
        add_key_value("path_publish_stride", std::to_string(path_publish_stride_));

        diag.status.push_back(status);
        debug_pub_.publish(diag);
    }

    void ProcessStereoPair(const cv::Mat& left_image,
                           const cv::Mat& right_image,
                           const ros::Time& stamp)
    {
        Eigen::Affine3d abs_pose = Eigen::Affine3d::Identity();
        std::vector<cv::Point2f> features;
        Vop2el::Vop2elAlgorithm::FrameDebugStats frame_stats;
        const bool need_features = publish_features_ || publish_features_image_ || publish_debug_;
        {
            std::lock_guard<std::mutex> lock(algorithm_mutex_);
            if (!camera_ready_ || !algorithm_)
                return;

            Eigen::Affine3d relative = Eigen::Affine3d::Identity();
            algorithm_->ProcessStereoFrame(left_image, right_image, relative);
            abs_pose = algorithm_->GetCurrentAbsPoseCopy();
            if (need_features)
                features = algorithm_->GetLatestLeftKeyPointsCopy();
            frame_stats = algorithm_->GetLastFrameDebugStats();
        }

        frames_processed_.fetch_add(1, std::memory_order_relaxed);
        ros::WallTime now_wall = ros::WallTime::now();
        if (has_last_process_wall_)
        {
            double dt = (now_wall - last_process_wall_).toSec();
            if (dt > 1e-6)
            {
                double inst_fps = 1.0 / dt;
                processing_fps_ = (processing_fps_ == 0.0) ? inst_fps : (0.9 * processing_fps_ + 0.1 * inst_fps);
            }
        }
        last_process_wall_ = now_wall;
        has_last_process_wall_ = true;

        if (skip_publish_on_fallback_ && frame_stats.UsedFallback)
        {
            frames_skipped_publish_fallback_.fetch_add(1, std::memory_order_relaxed);
            PublishDebug(stamp, features.size(), published_poses_.size(),
                         frame_stats.MatchCount, frame_stats.InlierCount,
                         frame_stats.UsedFallback, frame_stats.UsedExtrapolation, frame_stats.FailureReason);
            return;
        }

        published_poses_.push_back(abs_pose);
        frames_published_.fetch_add(1, std::memory_order_relaxed);

        if (publish_odom_)
        {
            nav_msgs::Odometry odom;
            odom.header.stamp = stamp;
            odom.header.frame_id = odom_frame_;
            odom.child_frame_id = base_frame_;
            odom.pose.pose = PoseFromEigen(abs_pose);
            odom_pub_.publish(odom);
        }

        if (publish_path_)
        {
            geometry_msgs::PoseStamped ps;
            ps.header.stamp = stamp;
            ps.header.frame_id = odom_frame_;
            ps.pose = PoseFromEigen(abs_pose);
            path_msg_.header.stamp = stamp;
            path_msg_.header.frame_id = odom_frame_;
            path_msg_.poses.push_back(ps);

            ++path_publish_counter_;
            if (path_msg_.poses.size() == 1 || (path_publish_counter_ % static_cast<uint64_t>(path_publish_stride_)) == 0)
                path_pub_.publish(path_msg_);
        }

        if (publish_tf_)
        {
            geometry_msgs::TransformStamped tf_msg;
            tf_msg.header.stamp = stamp;
            tf_msg.header.frame_id = odom_frame_;
            tf_msg.child_frame_id = base_frame_;
            tf_msg.transform = TransformFromEigen(abs_pose);
            tf_broadcaster_.sendTransform(tf_msg);
        }

        PublishFeatures(features, stamp);
        PublishFeaturesImage(left_image, features, stamp);
        PublishDebug(stamp, features.size(), published_poses_.size(),
                     frame_stats.MatchCount, frame_stats.InlierCount,
                     frame_stats.UsedFallback, frame_stats.UsedExtrapolation, frame_stats.FailureReason);
    }

    void ProcessingLoop()
    {
        while (ros::ok())
        {
            BufferedStereoFrame frame;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this]() { return !worker_running_ || !frame_queue_.empty(); });
                if (!worker_running_ && frame_queue_.empty())
                    break;

                frame = std::move(frame_queue_.front());
                frame_queue_.pop_front();
            }

            ProcessStereoPair(frame.left, frame.right, frame.stamp);
        }
    }

    void StopWorker()
    {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            worker_running_ = false;
        }
        queue_cv_.notify_all();
        if (worker_thread_.joinable())
            worker_thread_.join();
    }

    void StereoWithInfoCallback(const sensor_msgs::ImageConstPtr& left_msg,
                                const sensor_msgs::ImageConstPtr& right_msg,
                                const sensor_msgs::CameraInfoConstPtr& left_info,
                                const sensor_msgs::CameraInfoConstPtr& right_info)
    {
        if (!camera_ready_)
        {
            if (!UpdateCameraParams(left_info, right_info))
                return;
            std::lock_guard<std::mutex> lock(algorithm_mutex_);
            algorithm_.reset(new Vop2el::Vop2elAlgorithm(params_));
            camera_ready_ = true;
        }
        EnqueueStereoPair(left_msg, right_msg);
    }

    void StereoImagesCallback(const sensor_msgs::ImageConstPtr& left_msg,
                              const sensor_msgs::ImageConstPtr& right_msg)
    {
        EnqueueStereoPair(left_msg, right_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vop2el_node");
    Vop2elNode node;
    ros::spin();
    return 0;
}
