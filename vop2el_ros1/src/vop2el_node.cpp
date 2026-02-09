#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
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

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>

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

        left_image_sub_.subscribe(it_, left_image_topic_, 1);
        right_image_sub_.subscribe(it_, right_image_topic_, 1);
        if (use_camera_info_)
        {
            left_info_sub_.subscribe(nh_, left_camera_info_topic_, 1);
            right_info_sub_.subscribe(nh_, right_camera_info_topic_, 1);
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
            sync_images_.reset(new SyncImages(SyncPolicyImages(queue_size_), left_image_sub_, right_image_sub_));
            sync_images_->registerCallback(boost::bind(&Vop2elNode::StereoImagesCallback, this,
                                                       boost::placeholders::_1, boost::placeholders::_2));
        }

        if (use_imu_)
        {
            const int imu_sub_queue = std::max(50, queue_size_ * 10);
            imu_sub_ = nh_.subscribe(imu_topic_, imu_sub_queue, &Vop2elNode::ImuCallback, this);
            ROS_INFO("IMU fusion enabled. imu_topic=%s weight=%.3f max_dt=%.3f",
                     imu_topic_.c_str(), imu_orientation_weight_, imu_max_time_diff_);
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
    using SyncPolicyImages = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                                            sensor_msgs::Image>;
    using SyncImages = message_filters::Synchronizer<SyncPolicyImages>;
    struct BufferedStereoFrame
    {
        cv::Mat left;
        cv::Mat right;
        ros::Time stamp;
    };

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;

    image_transport::SubscriberFilter left_image_sub_;
    image_transport::SubscriberFilter right_image_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> right_info_sub_;
    ros::Subscriber imu_sub_;
    std::unique_ptr<SyncWithInfo> sync_with_info_;
    std::unique_ptr<SyncImages> sync_images_;

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
    std::string imu_topic_;
    std::string imu_frame_override_;
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
    bool use_imu_ = false;
    bool use_rectified_ = true;
    double tf_lookup_timeout_ = 0.1;
    double imu_max_time_diff_ = 0.03;
    double imu_orientation_weight_ = 0.2;
    int queue_size_ = 10;
    int input_buffer_size_ = 5;
    int imu_queue_size_ = 400;
    bool drop_oldest_when_full_ = true;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<BufferedStereoFrame> frame_queue_;
    bool worker_running_ = true;
    std::thread worker_thread_;
    std::atomic<uint64_t> frames_enqueued_{0};
    std::atomic<uint64_t> frames_dropped_{0};
    std::atomic<uint64_t> frames_processed_{0};
    ros::WallTime last_process_wall_;
    bool has_last_process_wall_ = false;
    double processing_fps_ = 0.0;
    std::mutex imu_mutex_;
    std::deque<sensor_msgs::ImuConstPtr> imu_queue_;
    bool imu_alignment_initialized_ = false;
    Eigen::Quaterniond imu_to_vo_orientation_offset_ = Eigen::Quaterniond::Identity();
    bool imu_to_base_rotation_cached_ = false;
    std::string imu_to_base_rotation_source_frame_;
    Eigen::Quaterniond imu_to_base_rotation_cached_q_ = Eigen::Quaterniond::Identity();
    ros::WallTime last_imu_tf_lookup_attempt_wall_;
    std::vector<Eigen::Affine3d> published_poses_;

    void LoadParams()
    {
        pnh_.param("left_image_topic", left_image_topic_, std::string("/stereo/left/image_rect"));
        pnh_.param("right_image_topic", right_image_topic_, std::string("/stereo/right/image_rect"));
        pnh_.param("left_camera_info_topic", left_camera_info_topic_, std::string("/stereo/left/camera_info"));
        pnh_.param("right_camera_info_topic", right_camera_info_topic_, std::string("/stereo/right/camera_info"));
        pnh_.param("imu_topic", imu_topic_, std::string("/imu/data"));
        pnh_.param("imu_frame_override", imu_frame_override_, std::string(""));
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
        pnh_.param("use_imu", use_imu_, false);
        pnh_.param("use_rectified", use_rectified_, true);
        pnh_.param("tf_lookup_timeout", tf_lookup_timeout_, 0.1);
        pnh_.param("imu_max_time_diff", imu_max_time_diff_, 0.03);
        pnh_.param("imu_orientation_weight", imu_orientation_weight_, 0.2);
        pnh_.param("queue_size", queue_size_, 10);
        pnh_.param("input_buffer_size", input_buffer_size_, 5);
        pnh_.param("imu_queue_size", imu_queue_size_, 400);
        pnh_.param("drop_oldest_when_full", drop_oldest_when_full_, true);

        if (!use_camera_info_)
        {
            const bool left_looks_color = left_image_topic_.find("camera_color") != std::string::npos;
            const bool right_looks_color = right_image_topic_.find("camera_color") != std::string::npos;
            if (left_looks_color || right_looks_color)
            {
                ROS_WARN("use_camera_info=false with color image topics; ensure INI/TXT intrinsics are for the same color cameras to avoid drift.");
            }
        }

        bool loaded_from_ini = false;
        if (!ini_file_.empty())
        {
            try
            {
                Utils::GenerateVop2elParamsFromIniFile(ini_file_, params_);
                loaded_from_ini = true;
                ROS_INFO("Loaded Vop2el parameters from ini_file: %s", ini_file_.c_str());
                ROS_INFO("When ini_file is set, VO tuning values are taken from INI/TXT calibration.");
            }
            catch (const std::exception& ex)
            {
                ROS_FATAL("Failed to load ini_file '%s': %s", ini_file_.c_str(), ex.what());
                throw;
            }
        }
        if (!loaded_from_ini && !use_camera_info_)
        {
            ROS_FATAL("use_camera_info is false but ini_file is empty. Provide ini_file with stereo camera parameters.");
            throw std::runtime_error("Missing ini_file while use_camera_info=false");
        }
        if (!loaded_from_ini)
        {
            int of_window_rows = 31;
            int of_window_cols = 31;
            int of_pyramid_level = 3;
            double of_eigen_threshold = 0.001;
            int of_criteria_max_count = 50;
            double of_criteria_epsilon = 0.05;
            double of_forward_backward_threshold = 2.0;

            pnh_.param("optical_flow/window_rows", of_window_rows, of_window_rows);
            pnh_.param("optical_flow/window_cols", of_window_cols, of_window_cols);
            pnh_.param("optical_flow/pyramid_level", of_pyramid_level, of_pyramid_level);
            pnh_.param("optical_flow/eigen_threshold", of_eigen_threshold, of_eigen_threshold);
            pnh_.param("optical_flow/criteria_max_count", of_criteria_max_count, of_criteria_max_count);
            pnh_.param("optical_flow/criteria_epsilon", of_criteria_epsilon, of_criteria_epsilon);
            pnh_.param("optical_flow/forward_backward_threshold", of_forward_backward_threshold, of_forward_backward_threshold);

            params_.OfWindowSize = cv::Size(of_window_cols, of_window_rows);
            params_.OfPyramidLevel = of_pyramid_level;
            params_.OfEigenTreshold = of_eigen_threshold;
            params_.OfForwardBackwardTreshold = static_cast<float>(of_forward_backward_threshold);
            params_.OfCriteria = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                                  of_criteria_max_count,
                                                  of_criteria_epsilon);

            int max_num_iterations = 500;
            bool use_tukey = true;
            double tukey_parameter = 1.0;
            pnh_.param("cost_functions/max_num_iterations", max_num_iterations, max_num_iterations);
            pnh_.param("cost_functions/use_tukey", use_tukey, use_tukey);
            pnh_.param("cost_functions/tukey_parameter", tukey_parameter, tukey_parameter);
            params_.CostFunctionsMaxNumIterations = max_num_iterations;
            params_.UseTukeyEstimator = use_tukey;
            params_.TukeyParameter = tukey_parameter;

            int max_number_matches = -1;
            double ncc_threshold = 0.7;
            int epipolar_line_search_interval = 100;
            int max_stereo_points_to_process = 10;
            int half_patch_rows = 4;
            int half_patch_cols = 4;
            int half_vertical_search = 4;
            int half_horizontal_search = 4;
            double max_thresh = 0.25;
            pnh_.param("matcher/max_number_matches", max_number_matches, max_number_matches);
            pnh_.param("matcher/ncc_threshold", ncc_threshold, ncc_threshold);
            pnh_.param("matcher/epipolar_line_search_interval", epipolar_line_search_interval, epipolar_line_search_interval);
            pnh_.param("matcher/max_stereo_points_to_process", max_stereo_points_to_process, max_stereo_points_to_process);
            pnh_.param("matcher/half_patch_rows", half_patch_rows, half_patch_rows);
            pnh_.param("matcher/half_patch_cols", half_patch_cols, half_patch_cols);
            pnh_.param("matcher/half_vertical_search", half_vertical_search, half_vertical_search);
            pnh_.param("matcher/half_horizontal_search", half_horizontal_search, half_horizontal_search);
            pnh_.param("matcher/max_thresh", max_thresh, max_thresh);

            params_.Vop2elMatcherParams.MaxNumberOfMatches = max_number_matches;
            params_.Vop2elMatcherParams.NccTreshold = static_cast<float>(ncc_threshold);
            params_.Vop2elMatcherParams.EpipolarLineSearchInterval = epipolar_line_search_interval;
            params_.Vop2elMatcherParams.MaxStereoPointsToProcess = max_stereo_points_to_process;
            params_.Vop2elMatcherParams.HalfPatchRows = half_patch_rows;
            params_.Vop2elMatcherParams.HalfPatchCols = half_patch_cols;
            params_.Vop2elMatcherParams.HalfVerticalSearch = half_vertical_search;
            params_.Vop2elMatcherParams.HalfHorizontalSearch = half_horizontal_search;
            params_.Vop2elMatcherParams.MaxThresh = static_cast<float>(max_thresh);

            int num_frames_capacity = 2;
            int bin_width = 50;
            int bin_height = 50;
            int max_key_points_per_bin = 3;
            pnh_.param("stereo_handler/num_frames_capacity", num_frames_capacity, num_frames_capacity);
            pnh_.param("stereo_handler/bin_width", bin_width, bin_width);
            pnh_.param("stereo_handler/bin_height", bin_height, bin_height);
            pnh_.param("stereo_handler/max_key_points_per_bin", max_key_points_per_bin, max_key_points_per_bin);

            params_.StereoImagesHandlerParams.NumFramesCapacity = num_frames_capacity;
            params_.StereoImagesHandlerParams.BinWidth = bin_width;
            params_.StereoImagesHandlerParams.BinHeight = bin_height;
            params_.StereoImagesHandlerParams.MaxNumberOfKeyPointsPerBin = max_key_points_per_bin;

            bool use_ground_plane = false;
            double plane_normal_x = 0.0;
            double plane_normal_y = -1.0;
            double plane_normal_z = 0.0;
            double plane_distance = 1.65;
            pnh_.param("ground_plane/use_ground_plane_correction", use_ground_plane, use_ground_plane);
            pnh_.param("ground_plane/plane_normal_x", plane_normal_x, plane_normal_x);
            pnh_.param("ground_plane/plane_normal_y", plane_normal_y, plane_normal_y);
            pnh_.param("ground_plane/plane_normal_z", plane_normal_z, plane_normal_z);
            pnh_.param("ground_plane/camera_ground_plane_distance", plane_distance, plane_distance);

            params_.PlaneNormal.reset();
            params_.PlaneDistance.reset();
            if (use_ground_plane)
            {
                params_.PlaneNormal.reset(new cv::Vec3f(static_cast<float>(plane_normal_x),
                                                        static_cast<float>(plane_normal_y),
                                                        static_cast<float>(plane_normal_z)));
                params_.PlaneDistance.reset(new float(static_cast<float>(plane_distance)));
            }
        }

        pnh_.param("sliding_window/enable", params_.EnableSlidingWindow, params_.EnableSlidingWindow);
        pnh_.param("sliding_window/size", params_.SlidingWindowSize, params_.SlidingWindowSize);
        pnh_.param("sliding_window/background", params_.SlidingWindowBackground, params_.SlidingWindowBackground);
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

    void ImuCallback(const sensor_msgs::ImuConstPtr& msg)
    {
        if (!use_imu_)
            return;

        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_queue_.push_back(msg);
        const int max_size = std::max(50, imu_queue_size_);
        while (static_cast<int>(imu_queue_.size()) > max_size)
            imu_queue_.pop_front();
    }

    bool GetClosestImuSample(const ros::Time& stamp,
                             sensor_msgs::ImuConstPtr& imu_msg,
                             double& dt_sec)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (imu_queue_.empty())
            return false;

        double best_abs_dt = std::numeric_limits<double>::infinity();
        sensor_msgs::ImuConstPtr best;
        for (const auto& sample : imu_queue_)
        {
            const double abs_dt = std::abs((sample->header.stamp - stamp).toSec());
            if (abs_dt < best_abs_dt)
            {
                best_abs_dt = abs_dt;
                best = sample;
            }
        }

        if (!best || best_abs_dt > imu_max_time_diff_)
            return false;

        imu_msg = best;
        dt_sec = best_abs_dt;
        return true;
    }

    bool ResolveImuToBaseRotation(const std::string& imu_frame,
                                  Eigen::Quaterniond& q_imu_to_base)
    {
        if (imu_frame == base_frame_)
        {
            q_imu_to_base = Eigen::Quaterniond::Identity();
            return true;
        }

        if (imu_to_base_rotation_cached_ && imu_to_base_rotation_source_frame_ == imu_frame)
        {
            q_imu_to_base = imu_to_base_rotation_cached_q_;
            return true;
        }

        const ros::WallTime now = ros::WallTime::now();
        if (!last_imu_tf_lookup_attempt_wall_.isZero() &&
            (now - last_imu_tf_lookup_attempt_wall_).toSec() < 0.2)
        {
            return false;
        }
        last_imu_tf_lookup_attempt_wall_ = now;

        geometry_msgs::TransformStamped tf_imu_to_base;
        try
        {
            tf_imu_to_base = tf_buffer_.lookupTransform(base_frame_, imu_frame,
                                                        ros::Time(0), ros::Duration(0.0));
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_THROTTLE(2.0, "IMU TF lookup failed (non-blocking): %s", ex.what());
            return false;
        }

        Eigen::Quaterniond q(tf_imu_to_base.transform.rotation.w,
                             tf_imu_to_base.transform.rotation.x,
                             tf_imu_to_base.transform.rotation.y,
                             tf_imu_to_base.transform.rotation.z);
        if (!std::isfinite(q.norm()) || q.norm() < 1e-9)
            return false;
        q.normalize();

        imu_to_base_rotation_cached_q_ = q;
        imu_to_base_rotation_source_frame_ = imu_frame;
        imu_to_base_rotation_cached_ = true;
        q_imu_to_base = q;
        return true;
    }

    bool TryFuseImuOrientation(const ros::Time& stamp,
                               Eigen::Affine3d& pose_in_out,
                               bool& imu_used,
                               double& imu_dt_sec)
    {
        imu_used = false;
        imu_dt_sec = -1.0;

        if (!use_imu_)
            return false;

        sensor_msgs::ImuConstPtr imu_msg;
        if (!GetClosestImuSample(stamp, imu_msg, imu_dt_sec))
            return false;

        if (imu_msg->orientation_covariance[0] < 0.0)
            return false;

        Eigen::Quaterniond q_world_imu(imu_msg->orientation.w,
                                       imu_msg->orientation.x,
                                       imu_msg->orientation.y,
                                       imu_msg->orientation.z);
        if (!std::isfinite(q_world_imu.norm()) || q_world_imu.norm() < 1e-9)
            return false;
        q_world_imu.normalize();

        std::string imu_frame = imu_frame_override_.empty() ? imu_msg->header.frame_id : imu_frame_override_;
        if (imu_frame.empty())
            imu_frame = base_frame_;

        Eigen::Quaterniond q_imu_to_base = Eigen::Quaterniond::Identity();
        if (!ResolveImuToBaseRotation(imu_frame, q_imu_to_base))
            return false;

        const Eigen::Quaterniond q_world_base_from_imu = q_world_imu * q_imu_to_base;

        Eigen::Quaterniond q_vo_world_base(pose_in_out.rotation());
        if (!std::isfinite(q_vo_world_base.norm()) || q_vo_world_base.norm() < 1e-9)
            return false;
        q_vo_world_base.normalize();

        if (!imu_alignment_initialized_)
        {
            imu_to_vo_orientation_offset_ = q_vo_world_base * q_world_base_from_imu.inverse();
            imu_alignment_initialized_ = true;
        }

        Eigen::Quaterniond q_imu_mapped = imu_to_vo_orientation_offset_ * q_world_base_from_imu;
        if (!std::isfinite(q_imu_mapped.norm()) || q_imu_mapped.norm() < 1e-9)
            return false;
        q_imu_mapped.normalize();

        const double w = std::max(0.0, std::min(1.0, imu_orientation_weight_));
        Eigen::Quaterniond q_fused = q_vo_world_base.slerp(w, q_imu_mapped);
        if (!std::isfinite(q_fused.norm()) || q_fused.norm() < 1e-9)
            return false;
        q_fused.normalize();

        pose_in_out.linear() = q_fused.toRotationMatrix();
        imu_used = true;
        return true;
    }

    void EnqueueStereoPair(const sensor_msgs::ImageConstPtr& left_msg,
                           const sensor_msgs::ImageConstPtr& right_msg)
    {
        {
            std::lock_guard<std::mutex> lock(algorithm_mutex_);
            if (!camera_ready_ || !algorithm_)
                return;
        }

        cv::Mat left_img;
        cv::Mat right_img;
        if (!ConvertImageMsg(left_msg, left_img) || !ConvertImageMsg(right_msg, right_img))
            return;

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

        frame_queue_.push_back(BufferedStereoFrame{left_img, right_img, left_msg->header.stamp});
        frames_enqueued_.fetch_add(1, std::memory_order_relaxed);
        lock.unlock();
        queue_cv_.notify_one();
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
                      bool imu_used,
                      double imu_dt_sec)
    {
        if (!publish_debug_ || !debug_pub_)
            return;

        size_t queue_depth = 0;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            queue_depth = frame_queue_.size();
        }
        size_t imu_queue_depth = 0;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            imu_queue_depth = imu_queue_.size();
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
        add_key_value("imu_queue_depth", std::to_string(imu_queue_depth));
        add_key_value("frames_enqueued", std::to_string(frames_enqueued_.load(std::memory_order_relaxed)));
        add_key_value("frames_processed", std::to_string(frames_processed_.load(std::memory_order_relaxed)));
        add_key_value("frames_dropped", std::to_string(frames_dropped_.load(std::memory_order_relaxed)));
        add_key_value("processing_fps", std::to_string(processing_fps_));
        add_key_value("feature_count", std::to_string(feature_count));
        add_key_value("match_count", std::to_string(match_count));
        add_key_value("inlier_count", std::to_string(inlier_count));
        add_key_value("pose_count", std::to_string(pose_count));
        add_key_value("use_camera_info", use_camera_info_ ? "true" : "false");
        add_key_value("use_imu", use_imu_ ? "true" : "false");
        add_key_value("imu_used", imu_used ? "true" : "false");
        add_key_value("imu_dt_sec", std::to_string(imu_dt_sec));
        add_key_value("imu_tf_cached", imu_to_base_rotation_cached_ ? "true" : "false");
        add_key_value("use_rectified", use_rectified_ ? "true" : "false");

        diag.status.push_back(status);
        debug_pub_.publish(diag);
    }

    void ProcessStereoPair(const cv::Mat& left_image,
                           const cv::Mat& right_image,
                           const ros::Time& stamp)
    {
        std::vector<Eigen::Affine3d> poses;
        std::vector<cv::Point2f> features;
        Vop2el::Vop2elAlgorithm::FrameDebugStats frame_stats;
        {
            std::lock_guard<std::mutex> lock(algorithm_mutex_);
            if (!camera_ready_ || !algorithm_)
                return;

            Eigen::Affine3d relative = Eigen::Affine3d::Identity();
            algorithm_->ProcessStereoFrame(left_image, right_image, relative);
            poses = algorithm_->GetPosesCopy();
            features = algorithm_->GetLatestLeftKeyPointsCopy();
            frame_stats = algorithm_->GetLastFrameDebugStats();
        }

        if (poses.empty())
            return;

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

        Eigen::Affine3d abs_pose = poses.back();
        bool imu_used = false;
        double imu_dt_sec = -1.0;
        TryFuseImuOrientation(stamp, abs_pose, imu_used, imu_dt_sec);
        published_poses_.push_back(abs_pose);

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
            nav_msgs::Path path;
            path.header.stamp = stamp;
            path.header.frame_id = odom_frame_;
            path.poses.reserve(published_poses_.size());
            for (const auto& pose : published_poses_)
            {
                geometry_msgs::PoseStamped ps;
                ps.header.stamp = stamp;
                ps.header.frame_id = odom_frame_;
                ps.pose = PoseFromEigen(pose);
                path.poses.push_back(ps);
            }
            path_pub_.publish(path);
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
                     imu_used, imu_dt_sec);
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
