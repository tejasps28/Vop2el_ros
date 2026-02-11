/*
 * Project: Vop2el
 *
 * Author: Mohamed Mssaouri
 *
 * Copyright (c) 2024 Mohamed Mssaouri
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <atomic>
#include <algorithm>
#include <cstdlib>

#include "Vop2elAlgorithm.h"
#include "Common.h"
#include "Vop2elCostStructs.h"

namespace Vop2el
{
namespace
{
int GetSolverNumThreads()
{
    static int configured_threads = []()
    {
        const char* env = std::getenv("VOP2EL_NUM_THREADS");
        if (env != nullptr)
        {
            const int parsed = std::atoi(env);
            if (parsed > 0)
                return parsed;
        }

        const unsigned int hw = std::thread::hardware_concurrency();
        if (hw == 0u)
            return 1;
        return static_cast<int>(std::min<unsigned int>(hw, 4u));
    }();
    return configured_threads;
}

void LogFallbackWarning(bool extrapolate_on_failure)
{
    static std::atomic<int> warning_counter{0};
    const int count = warning_counter.fetch_add(1, std::memory_order_relaxed) + 1;
    if (count <= 5 || (count % 50) == 0)
    {
        std::cerr << "[WARNING] The number of computed matches is insufficient. "
                  << (extrapolate_on_failure ? "Applying last-motion extrapolation." : "Holding pose (identity fallback).")
                  << std::endl;
        if (count == 5)
            std::cerr << "[WARNING] Suppressing frequent fallback warnings; logging every 50 events." << std::endl;
    }
}

struct RelativePoseCost
{
    RelativePoseCost(const Eigen::Quaterniond& q_meas, const Eigen::Vector3d& t_meas)
        : q_meas_(q_meas), t_meas_(t_meas) {}

    template <typename T>
    bool operator()(const T* const qi, const T* const ti,
                    const T* const qj, const T* const tj,
                    T* residual) const
    {
        Eigen::Quaternion<T> Q_i(qi[0], qi[1], qi[2], qi[3]);
        Eigen::Quaternion<T> Q_j(qj[0], qj[1], qj[2], qj[3]);
        Eigen::Matrix<T, 3, 1> t_i(ti[0], ti[1], ti[2]);
        Eigen::Matrix<T, 3, 1> t_j(tj[0], tj[1], tj[2]);

        Eigen::Quaternion<T> q_rel = Q_i.conjugate() * Q_j;
        Eigen::Matrix<T, 3, 1> t_rel = Q_i.conjugate() * (t_j - t_i);

        Eigen::Quaternion<T> q_meas = q_meas_.cast<T>();
        Eigen::Matrix<T, 3, 1> t_meas = t_meas_.cast<T>();

        Eigen::Quaternion<T> q_err = q_meas.conjugate() * q_rel;
        if (q_err.w() < T(0))
            q_err.coeffs() = -q_err.coeffs();

        residual[0] = t_rel(0) - t_meas(0);
        residual[1] = t_rel(1) - t_meas(1);
        residual[2] = t_rel(2) - t_meas(2);
        residual[3] = T(2.0) * q_err.x();
        residual[4] = T(2.0) * q_err.y();
        residual[5] = T(2.0) * q_err.z();
        return true;
    }

    Eigen::Quaterniond q_meas_;
    Eigen::Vector3d t_meas_;
};
}
//-------------------------------------------------------------------------------------------
void Vop2elAlgorithm::EstimateInitMatchesUsingOF(std::shared_ptr<const cv::Mat> refImage,
                                                std::shared_ptr<const cv::Mat> tarImage,
                                                std::shared_ptr<const std::vector<cv::Point2f>> refImageKeyPoints,
                                                std::vector<cv::Point2f>& refMatchesKeyPoints,
                                                std::vector<cv::Point2f>& tarMatchesKeyPoints) const
{
    refMatchesKeyPoints.clear();
    tarMatchesKeyPoints.clear();
    std::vector<uchar> statusForward;
    std::vector<float> errorsForward;
    std::vector<cv::Point2f> actualKeyPoints;
    cv::calcOpticalFlowPyrLK(*refImage, *tarImage, *refImageKeyPoints, actualKeyPoints, statusForward, errorsForward,
                            this->Vop2elParams.OfWindowSize, this->Vop2elParams.OfPyramidLevel, this->Vop2elParams.OfCriteria,
                            0, this->Vop2elParams.OfEigenTreshold);

    std::vector<uchar> statusBackward;
    std::vector<float> errorsBackward;
    std::vector<cv::Point2f> refKeyPointsBack;
    cv::calcOpticalFlowPyrLK(*tarImage, *refImage, actualKeyPoints, refKeyPointsBack, statusBackward, errorsBackward,
                            this->Vop2elParams.OfWindowSize, this->Vop2elParams.OfPyramidLevel, this->Vop2elParams.OfCriteria,
                            0, this->Vop2elParams.OfEigenTreshold);

    for (int keyPointIdx = 0; keyPointIdx < refImageKeyPoints->size(); ++keyPointIdx)
    {
        cv::Vec2f forwardVect(actualKeyPoints[keyPointIdx] - (*refImageKeyPoints)[keyPointIdx]);
        cv::Vec2f backwardVect(refKeyPointsBack[keyPointIdx] - actualKeyPoints[keyPointIdx]);
        if ((cv::norm(forwardVect + backwardVect) < this->Vop2elParams.OfForwardBackwardTreshold))
        {
            refMatchesKeyPoints.push_back(((*refImageKeyPoints)[keyPointIdx]));
            tarMatchesKeyPoints.push_back(actualKeyPoints[keyPointIdx]);
        }
    }
}

//-------------------------------------------------------------------------------------------
void Vop2elAlgorithm::EstimateInitScalessRelativeTransform(Eigen::Affine3d& transformPreviousActual) const
{
    std::vector<cv::Point2f> validPreviousLeftKeyPoints;
    std::vector<cv::Point2f> validActualLeftKeyPoints;

    std::shared_ptr<const cv::Mat> previousLeftImage = this->FramesHandler.GetLeftImage(-2);
    std::shared_ptr<const cv::Mat> actualLeftImage = this->FramesHandler.GetLeftImage(-1);
    std::shared_ptr<const std::vector<cv::Point2f>> previousLeftKeyPoints = this->FramesHandler.GetLeftImageKeyPoints(-2);

    this->EstimateInitMatchesUsingOF(previousLeftImage, actualLeftImage, previousLeftKeyPoints,
                                    validPreviousLeftKeyPoints, validActualLeftKeyPoints);

    cv::Mat maskInliers;
    cv::Mat essentialMatrix = cv::findEssentialMat(validActualLeftKeyPoints, validPreviousLeftKeyPoints,
                                                 this->Vop2elParams.CameraParams.CalibrationMatrix, cv::RANSAC, 0.999, 1 , maskInliers);

    cv::Mat relativeRotation, relativeTranslation;
    cv::recoverPose(essentialMatrix, validActualLeftKeyPoints, validPreviousLeftKeyPoints,
                    this->Vop2elParams.CameraParams.CalibrationMatrix, relativeRotation, relativeTranslation);

    Eigen::Matrix3dRowMajor rotationAfterOptimization(reinterpret_cast<double*>(relativeRotation.data));
    this->CheckRotation(rotationAfterOptimization);
    transformPreviousActual.linear() = rotationAfterOptimization;
    transformPreviousActual.translation() = Eigen::Vector3d(reinterpret_cast<double*>(relativeTranslation.data));
}

//-------------------------------------------------------------------------------------------
double Vop2elAlgorithm::EstimateInitScale(const Eigen::Affine3d& transformPreviousActual) const
{
    std::shared_ptr<const cv::Mat> previousLeftImage = this->FramesHandler.GetLeftImage(-2);
    std::shared_ptr<const cv::Mat> actualLeftImage = this->FramesHandler.GetLeftImage(-1);
    std::shared_ptr<const cv::Mat> previousRightImage = this->FramesHandler.GetRightImage(-2);
    std::shared_ptr<const cv::Mat> actualRightImage = this->FramesHandler.GetRightImage(-1);

    std::shared_ptr<const std::vector<cv::Point2f>> previousLeftKeyPoints = this->FramesHandler.GetLeftImageKeyPoints(-2);
    std::shared_ptr<const std::vector<cv::Point2f>> actualLeftKeyPoints = this->FramesHandler.GetLeftImageKeyPoints(-1);
    std::shared_ptr<const std::vector<cv::Point2f>> previousRightKeyPoints = this->FramesHandler.GetRightImageKeyPoints(-2);

    std::vector<cv::Point2f> refPreviousLeftActualRight;
    std::vector<cv::Point2f> tarPreviousLeftActualRight;
    this->EstimateInitMatchesUsingOF(previousLeftImage, actualRightImage, previousLeftKeyPoints,
                            refPreviousLeftActualRight, tarPreviousLeftActualRight);

    std::vector<cv::Point2f> refActualLeftPreviousRight;
    std::vector<cv::Point2f> tarActualLeftPreviousRight;
    this->EstimateInitMatchesUsingOF(actualLeftImage, previousRightImage, actualLeftKeyPoints,
                            refActualLeftPreviousRight, tarActualLeftPreviousRight);

    std::vector<cv::Point2f> refPreviousRightActualRight;
    std::vector<cv::Point2f> tarPreviousRightActualRight;
    this->EstimateInitMatchesUsingOF(previousRightImage, actualRightImage, previousRightKeyPoints,
                            refPreviousRightActualRight, tarPreviousRightActualRight);

    Eigen::Affine3d extrinsicTransformCamera =  Eigen::Affine3d::Identity();
    extrinsicTransformCamera.translation() = Eigen::Vector3d(reinterpret_cast<double*>(this->Vop2elParams.CameraParams.ExtrinsicTranslation.data));
    Eigen::Matrix3dRowMajor calibrationMatrix(reinterpret_cast<double*>(this->Vop2elParams.CameraParams.CalibrationMatrix.data));

    ceres::LossFunction* initScaleLossFunction = nullptr;
    if (this->Vop2elParams.UseTukeyEstimator)
       initScaleLossFunction = new ceres::TukeyLoss(this->Vop2elParams.TukeyParameter);

    double scale = 1.0;
    ceres::Problem problem;
    for (int keyPointIdx = 0; keyPointIdx < refPreviousLeftActualRight.size(); ++keyPointIdx)
    {
        cv::Point2d prevLeftPoint(refPreviousLeftActualRight[keyPointIdx]);
        cv::Point2d actRightPoint(tarPreviousLeftActualRight[keyPointIdx]);
        Eigen::Vector3d prevLeft(prevLeftPoint.x, prevLeftPoint.y, 1.0);
        Eigen::Vector3d actRight(actRightPoint.x, actRightPoint.y, 1.0);

        ceres::CostFunction* costPreviousLeftActualRight = new ceres::AutoDiffCostFunction<Vop2el::ScaleCostPreviousLeftActualRight, 2, 1>(
            new Vop2el::ScaleCostPreviousLeftActualRight(prevLeft, actRight, transformPreviousActual, extrinsicTransformCamera, calibrationMatrix));
        problem.AddResidualBlock(costPreviousLeftActualRight, initScaleLossFunction, &scale);
    }

    for (int keyPointIdx = 0; keyPointIdx < refActualLeftPreviousRight.size(); ++keyPointIdx)
    {
        cv::Point2d actLeftPoint(refActualLeftPreviousRight[keyPointIdx]);
        cv::Point2d prevRightPoint(tarActualLeftPreviousRight[keyPointIdx]);
        Eigen::Vector3d actLeft(actLeftPoint.x, actLeftPoint.y, 1.0);
        Eigen::Vector3d prevRight(prevRightPoint.x, prevRightPoint.y, 1.0);

        ceres::CostFunction* costActualLeftPreviousRight = new ceres::AutoDiffCostFunction<Vop2el::ScaleCostActualLeftPreviousRight, 2, 1>(
            new Vop2el::ScaleCostActualLeftPreviousRight(actLeft, prevRight, transformPreviousActual, extrinsicTransformCamera, calibrationMatrix));
        problem.AddResidualBlock(costActualLeftPreviousRight, initScaleLossFunction, &scale);
    }

    for (int keyPointIdx = 0; keyPointIdx < refPreviousRightActualRight.size(); ++keyPointIdx)
    {
        cv::Point2d prevRightPoint(refPreviousRightActualRight[keyPointIdx]);
        cv::Point2d actRightPoint(tarPreviousRightActualRight[keyPointIdx]);
        Eigen::Vector3d prevRight(prevRightPoint.x, prevRightPoint.y, 1.0);
        Eigen::Vector3d actRight(actRightPoint.x, actRightPoint.y, 1.0);

        ceres::CostFunction* costPreviousRightActualRight = new ceres::AutoDiffCostFunction<Vop2el::ScaleCostPreviousRightActualRight, 2, 1>(
            new Vop2el::ScaleCostPreviousRightActualRight(prevRight, actRight, transformPreviousActual, extrinsicTransformCamera, calibrationMatrix));
        problem.AddResidualBlock(costPreviousRightActualRight, initScaleLossFunction, &scale);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.num_threads = GetSolverNumThreads();
    ceres::Solver::Summary summary;
    options.max_num_iterations = this->Vop2elParams.CostFunctionsMaxNumIterations;
    ceres::Solve(options, &problem, &summary);

    return scale;
}

//-------------------------------------------------------------------------------------------
void Vop2elAlgorithm::CheckRotation(Eigen::Matrix3dRowMajor& rotationFromDecomposition) const
{
    Eigen::AngleAxisd axisAngle(rotationFromDecomposition);

    if (std::abs(axisAngle.angle() - M_PI) < 0.1)
    {
        axisAngle.angle() =  axisAngle.angle() - M_PI;
        rotationFromDecomposition = axisAngle.toRotationMatrix();
    }
}

//-------------------------------------------------------------------------------------------
void Vop2elAlgorithm::OptimizeEssentielMatrix(const cv::Mat& essentielMatrix,
                                            const std::vector<cv::Point2f>& prevPoints,
                                            const std::vector<cv::Point2f>& actPoints,
                                            const std::vector<bool>& maskInliers,
                                            cv::Mat& optimizedEssentielMatrix) const
{
    if ((prevPoints.size() != maskInliers.size()) || (actPoints.size() != maskInliers.size()))
        std::runtime_error("[ERROR] prevPoints, actPoints and maskInliers must have the same size");

    optimizedEssentielMatrix = essentielMatrix.clone();
    Eigen::Matrix3dRowMajor calibrationMatrixEigen(reinterpret_cast<const double*>(this->Vop2elParams.CameraParams.CalibrationMatrix.data));

    ceres::LossFunction* essentielMatLossFunction = nullptr;
    if (this->Vop2elParams.UseTukeyEstimator)
       essentielMatLossFunction = new ceres::TukeyLoss(this->Vop2elParams.TukeyParameter);

    ceres::Problem problem;
    for (int keyPointIdx = 0; keyPointIdx < maskInliers.size(); ++keyPointIdx)
    {
        if (maskInliers[keyPointIdx])
        {
            Eigen::Vector3d prevPoint(static_cast<double>(prevPoints[keyPointIdx].x), static_cast<double>(prevPoints[keyPointIdx].y), 1.0);
            Eigen::Vector3d actPoint(static_cast<double>(actPoints[keyPointIdx].x), static_cast<double>(actPoints[keyPointIdx].y), 1.0);

            ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<Vop2el::EssentielMatrixOptimizer, 2, 9>(
                new Vop2el::EssentielMatrixOptimizer(prevPoint, actPoint, calibrationMatrixEigen));
            problem.AddResidualBlock(costFunction, essentielMatLossFunction, reinterpret_cast<double*>(optimizedEssentielMatrix.data));
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.num_threads = GetSolverNumThreads();
    options.max_num_iterations = this->Vop2elParams.CostFunctionsMaxNumIterations;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
}

//-------------------------------------------------------------------------------------------
void Vop2elAlgorithm::ComputeScalessRelativeTransform(const std::vector<Vop2el::Match>& matches,
                                                    Eigen::Affine3d& relativeTransform,
                                                    std::vector<Vop2el::Match>& inliers) const
{
    inliers.clear();

    std::vector<cv::Point2f> prevPoints, actPoints;
    for (const auto& match : matches)
    {
        prevPoints.push_back(match.PreviousLeft);
        actPoints.push_back(match.ActualLeft);
    }

    cv::Mat maskInliers;
    cv::Mat essentielMatrix;
    essentielMatrix = cv::findEssentialMat(actPoints, prevPoints, this->Vop2elParams.CameraParams.CalibrationMatrix,
                                        cv::RANSAC, 0.999, 1, maskInliers);

    if (maskInliers.empty())
        throw std::runtime_error("[ERROR] findEssentialMat returned an empty inlier mask.");

    cv::Mat maskContinuous = maskInliers.isContinuous() ? maskInliers : maskInliers.clone();
    cv::Mat inliersMask = maskContinuous.reshape(1, static_cast<int>(maskContinuous.total())).clone();

    std::vector<bool> keyPointsInliersStatus(matches.size(), false);
    int inlierCount = std::min(static_cast<int>(matches.size()), inliersMask.rows);
    for (int keyPointIdx = 0; keyPointIdx < inlierCount; ++keyPointIdx)
    {
        if (inliersMask.at<unsigned char>(keyPointIdx, 0) != 0)
        {
            keyPointsInliersStatus[keyPointIdx] = true;
            inliers.push_back(matches[keyPointIdx]);
        }
    }

    cv::Mat optimizedEssentielMatrix;
    this->OptimizeEssentielMatrix(essentielMatrix, prevPoints, actPoints, keyPointsInliersStatus, optimizedEssentielMatrix);

    cv::Mat finalRotation, finalTranslation;
    cv::recoverPose(optimizedEssentielMatrix, actPoints, prevPoints, this->Vop2elParams.CameraParams.CalibrationMatrix,
                    finalRotation, finalTranslation, inliersMask);

    Eigen::Matrix3dRowMajor rotationAfterOptimization(reinterpret_cast<double*>(finalRotation.data));
    this->CheckRotation(rotationAfterOptimization);
    relativeTransform.linear() = rotationAfterOptimization;
    relativeTransform.translation() = Eigen::Vector3d(reinterpret_cast<double*>(finalTranslation.data));
}

//-------------------------------------------------------------------------------------------
double Vop2elAlgorithm::ComputeScale(const Eigen::Affine3d& transformPreviousActual,
                                    const std::vector<Vop2el::Match>& matches) const
{
    double scale = 1;

    if (!this->RelativePoses.empty())
        scale = this->RelativePoses.back().translation().norm();

    Eigen::Affine3d extrinsicTransformCamera =  Eigen::Affine3d::Identity();
    extrinsicTransformCamera.translation() = Eigen::Vector3d(reinterpret_cast<double*>(this->Vop2elParams.CameraParams.ExtrinsicTranslation.data));
    Eigen::Matrix3dRowMajor calibrationMatrix(reinterpret_cast<double*>(this->Vop2elParams.CameraParams.CalibrationMatrix.data));

    ceres::LossFunction* scaleLossFunction = nullptr;
    if (this->Vop2elParams.UseTukeyEstimator)
       scaleLossFunction = new ceres::TukeyLoss(this->Vop2elParams.TukeyParameter);

    ceres::Problem problem;
    for (const auto& match : matches)
    {
        Eigen::Vector3d prevLeft(static_cast<double>(match.PreviousLeft.x), static_cast<double>(match.PreviousLeft.y), 1.0);
        Eigen::Vector3d actLeft(static_cast<double>(match.ActualLeft.x), static_cast<double>(match.ActualLeft.y), 1.0);
        Eigen::Vector3d prevRight(static_cast<double>(match.PreviousRight.x), static_cast<double>(match.PreviousRight.y), 1.0);
        Eigen::Vector3d actRight(static_cast<double>(match.ActualRight.x), static_cast<double>(match.ActualRight.y), 1.0);

        ceres::CostFunction* costPreviousLeftActualRight = new ceres::AutoDiffCostFunction<Vop2el::ScaleCostPreviousLeftActualRight, 2, 1>(
            new Vop2el::ScaleCostPreviousLeftActualRight(prevLeft, actRight, transformPreviousActual, extrinsicTransformCamera, calibrationMatrix));
        problem.AddResidualBlock(costPreviousLeftActualRight, scaleLossFunction, &scale);

        ceres::CostFunction* costActualLeftPreviousRight = new ceres::AutoDiffCostFunction<Vop2el::ScaleCostActualLeftPreviousRight, 2, 1>(
            new Vop2el::ScaleCostActualLeftPreviousRight(actLeft, prevRight, transformPreviousActual, extrinsicTransformCamera, calibrationMatrix));
        problem.AddResidualBlock(costActualLeftPreviousRight, scaleLossFunction, &scale);

        ceres::CostFunction* costPreviousRightActualRight = new ceres::AutoDiffCostFunction<Vop2el::ScaleCostPreviousRightActualRight, 2, 1>(
            new Vop2el::ScaleCostPreviousRightActualRight(prevRight, actRight, transformPreviousActual, extrinsicTransformCamera, calibrationMatrix));
        problem.AddResidualBlock(costPreviousRightActualRight, scaleLossFunction, &scale);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.num_threads = GetSolverNumThreads();
    ceres::Solver::Summary summary;
    options.max_num_iterations = this->Vop2elParams.CostFunctionsMaxNumIterations;
    ceres::Solve(options, &problem, &summary);

    return scale;
}

//-------------------------------------------------------------------------------------------
void Vop2elAlgorithm::ComputeMatchesUsingVop2elMatcher(const Vop2el::StereoImagesPairWithKeyPoints& imagesWithKeyPoints,
                                                    const Eigen::Affine3d& previousActualTransform,
                                                    std::vector<Vop2el::Match>& matches,
                                                    int& NumberFixedKeyPoints) const
{
    matches.clear();

    cv::Mat cvPreviousActualTransform(4, 4, CV_64F);
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            cvPreviousActualTransform.at<double>(row, col) = previousActualTransform(row, col);

    cv::Affine3d cvAffinePreviousActual(cvPreviousActualTransform);
    Vop2el::Vop2elMatcher matcher(imagesWithKeyPoints, this->Vop2elParams.Vop2elMatcherParams, this->Vop2elParams.CameraParams, cvAffinePreviousActual);
    if (this->Vop2elParams.PlaneNormal && this->Vop2elParams.PlaneDistance)
        matcher.SetPlaneParameters(*(this->Vop2elParams.PlaneNormal), *(this->Vop2elParams.PlaneDistance));

    matcher.GetMatches(matches);
    NumberFixedKeyPoints = matcher.GetNumberFixedKeyPoints();
}

//-------------------------------------------------------------------------------------------
void Vop2elAlgorithm::EstimateInitScaledRelativeTransform(Eigen::Affine3d& initialRelativeTransform) const
{
    initialRelativeTransform = Eigen::Affine3d::Identity();
    this->EstimateInitScalessRelativeTransform(initialRelativeTransform);

    double initialScale = 0;
    if (this->RelativePoses.size() == 0)
        initialScale = this->EstimateInitScale(initialRelativeTransform);
    else
        initialScale = this->RelativePoses.back().translation().norm();

    initialRelativeTransform.translation() = initialScale * initialRelativeTransform.translation();
}

//-------------------------------------------------------------------------------------------
bool Vop2elAlgorithm::ProcessNumMatchesInsufficient()
{
    LogFallbackWarning(this->Vop2elParams.ExtrapolateOnFailure);
    std::lock_guard<std::mutex> lock(this->PosesMutex);
    Eigen::Affine3d fallbackRelative = Eigen::Affine3d::Identity();
    bool usedExtrapolation = false;
    if (this->Vop2elParams.ExtrapolateOnFailure && !this->RelativePoses.empty())
    {
        fallbackRelative = this->RelativePoses.back();
        usedExtrapolation = true;
    }

    this->RelativePoses.push_back(fallbackRelative);
    Eigen::Affine3d absolutePose = this->AbsolutePoses.back() * fallbackRelative;
    this->AbsolutePoses.push_back(absolutePose);
    this->PushPoseToWindow(absolutePose, &fallbackRelative);
    return usedExtrapolation;
}

//-------------------------------------------------------------------------------------------
void Vop2elAlgorithm::ProcessStereoFrame(const std::string& leftImage,
                                        const std::string& rightImage,
                                        Eigen::Affine3d& relativeTransform)
{
    if (this->FrameIndex < 2)
        this->FramesHandler.AddStereoPair(leftImage, rightImage);
    else
        this->FramesHandler.AddStereoPair(leftImage, rightImage, true, false);

    if (this->FrameIndex == 0)
    {
       this->SetLastFrameDebugStats(0, 0, false, false, 0);
       {
           std::lock_guard<std::mutex> lock(this->PosesMutex);
           Eigen::Affine3d origin = Eigen::Affine3d::Identity();
           this->AbsolutePoses.push_back(origin);
           this->PushPoseToWindow(origin, nullptr);
       }
       ++this->FrameIndex;
    }
    else
    {
        Vop2el::StereoImagesPairWithKeyPoints StereoImagesPairWithKeyPoints{
                                                this->FramesHandler.GetLeftImage(-2), this->FramesHandler.GetRightImage(-2),
                                                this->FramesHandler.GetLeftImage(-1), this->FramesHandler.GetRightImage(-1),
                                                this->FramesHandler.GetLeftImageKeyPoints(-1)};

        Eigen::Affine3d initialRelativeTransform;
        std::vector<Vop2el::Match> matches;
        int NumberFixedKeyPoints = 0;
        try
        {
            this->EstimateInitScaledRelativeTransform(initialRelativeTransform);
            this->ComputeMatchesUsingVop2elMatcher(StereoImagesPairWithKeyPoints, initialRelativeTransform, matches, NumberFixedKeyPoints);
        }
        catch (const cv::Exception& ex)
        {
            std::cerr << "[ERROR] OpenCV exception during initialization/matching: " << ex.what() << std::endl;
            const bool usedExtrapolation = this->ProcessNumMatchesInsufficient();
            this->SetLastFrameDebugStats(static_cast<int>(matches.size()), 0, true, usedExtrapolation, 4);
            this->MaybeOptimizeSlidingWindow();
            return;
        }

        if (matches.size() < 10)
        {
            const bool usedExtrapolation = this->ProcessNumMatchesInsufficient();
            this->SetLastFrameDebugStats(static_cast<int>(matches.size()), 0, true, usedExtrapolation, 1);
            this->MaybeOptimizeSlidingWindow();
            return;
        }

        double ratioOfFixedKeyPoints = static_cast<double>(NumberFixedKeyPoints) / static_cast<double>(matches.size());
        if (ratioOfFixedKeyPoints > 0.6)
        {
            {
                std::lock_guard<std::mutex> lock(this->PosesMutex);
                this->RelativePoses.push_back(Eigen::Affine3d::Identity());
                this->AbsolutePoses.push_back(this->AbsolutePoses.back());
            }
            this->SetLastFrameDebugStats(static_cast<int>(matches.size()), 0, true, false, 3);
            relativeTransform = Eigen::Affine3d::Identity();
            ++this->FrameIndex;
            this->MaybeOptimizeSlidingWindow();
            return;
        }

        Eigen::Affine3d scalessTransform = Eigen::Affine3d::Identity();
        std::vector<Vop2el::Match> inliers;
        double scale = 1.0;
        try
        {
            this->ComputeScalessRelativeTransform(matches, scalessTransform, inliers);
            if (inliers.size() < 10)
            {
                const bool usedExtrapolation = this->ProcessNumMatchesInsufficient();
                this->SetLastFrameDebugStats(static_cast<int>(matches.size()), static_cast<int>(inliers.size()),
                                             true, usedExtrapolation, 2);
                this->MaybeOptimizeSlidingWindow();
                return;
            }
            scale = this->ComputeScale(scalessTransform, inliers);
        }
        catch (const cv::Exception& ex)
        {
            std::cerr << "[ERROR] OpenCV exception during motion refinement: " << ex.what() << std::endl;
            const bool usedExtrapolation = this->ProcessNumMatchesInsufficient();
            this->SetLastFrameDebugStats(static_cast<int>(matches.size()), static_cast<int>(inliers.size()),
                                         true, usedExtrapolation, 5);
            this->MaybeOptimizeSlidingWindow();
            return;
        }

        Eigen::Affine3d scaledTransform = scalessTransform;
        scaledTransform.translation() = scale * scalessTransform.translation();
        {
            std::lock_guard<std::mutex> lock(this->PosesMutex);
            this->RelativePoses.push_back(scaledTransform);
        }
        relativeTransform = scaledTransform;

        Eigen::Affine3d absolutePose = Eigen::Affine3d::Identity();
        {
            std::lock_guard<std::mutex> lock(this->PosesMutex);
            absolutePose = this->AbsolutePoses.back() * scaledTransform;
            this->AbsolutePoses.push_back(absolutePose);
            this->PushPoseToWindow(absolutePose, &scaledTransform);
        }
        this->SetLastFrameDebugStats(static_cast<int>(matches.size()), static_cast<int>(inliers.size()),
                                     false, false, 0);

        ++this->FrameIndex;
        this->MaybeOptimizeSlidingWindow();
    }
}

//-------------------------------------------------------------------------------------------
void Vop2elAlgorithm::ProcessStereoFrame(const cv::Mat& leftImage,
                                        const cv::Mat& rightImage,
                                        Eigen::Affine3d& relativeTransform)
{
    if (this->FrameIndex < 2)
        this->FramesHandler.AddStereoPair(leftImage, rightImage);
    else
        this->FramesHandler.AddStereoPair(leftImage, rightImage, true, false);

    if (this->FrameIndex == 0)
    {
       this->SetLastFrameDebugStats(0, 0, false, false, 0);
       {
           std::lock_guard<std::mutex> lock(this->PosesMutex);
           Eigen::Affine3d origin = Eigen::Affine3d::Identity();
           this->AbsolutePoses.push_back(origin);
           this->PushPoseToWindow(origin, nullptr);
       }
       ++this->FrameIndex;
    }
    else
    {
        Vop2el::StereoImagesPairWithKeyPoints StereoImagesPairWithKeyPoints{
                                                this->FramesHandler.GetLeftImage(-2), this->FramesHandler.GetRightImage(-2),
                                                this->FramesHandler.GetLeftImage(-1), this->FramesHandler.GetRightImage(-1),
                                                this->FramesHandler.GetLeftImageKeyPoints(-1)};

        Eigen::Affine3d initialRelativeTransform;
        std::vector<Vop2el::Match> matches;
        int NumberFixedKeyPoints = 0;
        try
        {
            this->EstimateInitScaledRelativeTransform(initialRelativeTransform);
            this->ComputeMatchesUsingVop2elMatcher(StereoImagesPairWithKeyPoints, initialRelativeTransform, matches, NumberFixedKeyPoints);
        }
        catch (const cv::Exception& ex)
        {
            std::cerr << "[ERROR] OpenCV exception during initialization/matching: " << ex.what() << std::endl;
            const bool usedExtrapolation = this->ProcessNumMatchesInsufficient();
            this->SetLastFrameDebugStats(static_cast<int>(matches.size()), 0, true, usedExtrapolation, 4);
            this->MaybeOptimizeSlidingWindow();
            return;
        }

        if (matches.size() < 10)
        {
            const bool usedExtrapolation = this->ProcessNumMatchesInsufficient();
            this->SetLastFrameDebugStats(static_cast<int>(matches.size()), 0, true, usedExtrapolation, 1);
            this->MaybeOptimizeSlidingWindow();
            return;
        }

        double ratioOfFixedKeyPoints = static_cast<double>(NumberFixedKeyPoints) / static_cast<double>(matches.size());
        if (ratioOfFixedKeyPoints > 0.6)
        {
            {
                std::lock_guard<std::mutex> lock(this->PosesMutex);
                this->RelativePoses.push_back(Eigen::Affine3d::Identity());
                this->AbsolutePoses.push_back(this->AbsolutePoses.back());
            }
            this->SetLastFrameDebugStats(static_cast<int>(matches.size()), 0, true, false, 3);
            relativeTransform = Eigen::Affine3d::Identity();
            ++this->FrameIndex;
            this->MaybeOptimizeSlidingWindow();
            return;
        }

        Eigen::Affine3d scalessTransform = Eigen::Affine3d::Identity();
        std::vector<Vop2el::Match> inliers;
        double scale = 1.0;
        try
        {
            this->ComputeScalessRelativeTransform(matches, scalessTransform, inliers);
            if (inliers.size() < 10)
            {
                const bool usedExtrapolation = this->ProcessNumMatchesInsufficient();
                this->SetLastFrameDebugStats(static_cast<int>(matches.size()), static_cast<int>(inliers.size()),
                                             true, usedExtrapolation, 2);
                this->MaybeOptimizeSlidingWindow();
                return;
            }
            scale = this->ComputeScale(scalessTransform, inliers);
        }
        catch (const cv::Exception& ex)
        {
            std::cerr << "[ERROR] OpenCV exception during motion refinement: " << ex.what() << std::endl;
            const bool usedExtrapolation = this->ProcessNumMatchesInsufficient();
            this->SetLastFrameDebugStats(static_cast<int>(matches.size()), static_cast<int>(inliers.size()),
                                         true, usedExtrapolation, 5);
            this->MaybeOptimizeSlidingWindow();
            return;
        }

        Eigen::Affine3d scaledTransform = scalessTransform;
        scaledTransform.translation() = scale * scalessTransform.translation();
        {
            std::lock_guard<std::mutex> lock(this->PosesMutex);
            this->RelativePoses.push_back(scaledTransform);
        }
        relativeTransform = scaledTransform;

        Eigen::Affine3d absolutePose = Eigen::Affine3d::Identity();
        {
            std::lock_guard<std::mutex> lock(this->PosesMutex);
            absolutePose = this->AbsolutePoses.back() * scaledTransform;
            this->AbsolutePoses.push_back(absolutePose);
            this->PushPoseToWindow(absolutePose, &scaledTransform);
        }
        this->SetLastFrameDebugStats(static_cast<int>(matches.size()), static_cast<int>(inliers.size()),
                                     false, false, 0);

        ++this->FrameIndex;
        this->MaybeOptimizeSlidingWindow();
    }
}

//-------------------------------------------------------------------------------------------
Vop2elAlgorithm::~Vop2elAlgorithm()
{
    if (this->OptimizerThread.joinable())
        this->OptimizerThread.join();
}

//-------------------------------------------------------------------------------------------
std::vector<Eigen::Affine3d> Vop2elAlgorithm::GetPosesCopy() const
{
    std::lock_guard<std::mutex> lock(this->PosesMutex);
    return this->AbsolutePoses;
}

//-------------------------------------------------------------------------------------------
std::vector<cv::Point2f> Vop2elAlgorithm::GetLatestLeftKeyPointsCopy() const
{
    if (this->FramesHandler.GetSize() == 0)
        return {};

    try
    {
        std::shared_ptr<const std::vector<cv::Point2f>> keypoints = this->FramesHandler.GetLeftImageKeyPoints(-1);
        if (!keypoints)
            return {};
        return *keypoints;
    }
    catch (const std::exception&)
    {
        return {};
    }
}

//-------------------------------------------------------------------------------------------
Vop2elAlgorithm::FrameDebugStats Vop2elAlgorithm::GetLastFrameDebugStats() const
{
    std::lock_guard<std::mutex> lock(this->StatsMutex);
    return this->LastFrameDebugStats;
}

//-------------------------------------------------------------------------------------------
Eigen::Affine3d Vop2elAlgorithm::GetCurrentAbsPose() const
{
    std::lock_guard<std::mutex> lock(this->PosesMutex);
    return this->AbsolutePoses.back();
}

//-------------------------------------------------------------------------------------------
void Vop2elAlgorithm::SetLastFrameDebugStats(int matchCount, int inlierCount, bool usedFallback, bool usedExtrapolation, int failureReason)
{
    std::lock_guard<std::mutex> lock(this->StatsMutex);
    this->LastFrameDebugStats.MatchCount = matchCount;
    this->LastFrameDebugStats.InlierCount = inlierCount;
    this->LastFrameDebugStats.UsedFallback = usedFallback;
    this->LastFrameDebugStats.UsedExtrapolation = usedExtrapolation;
    this->LastFrameDebugStats.FailureReason = failureReason;
}

//-------------------------------------------------------------------------------------------
void Vop2elAlgorithm::PushPoseToWindow(const Eigen::Affine3d& absolutePose, const Eigen::Affine3d* relativePose)
{
    this->WindowPoses.push_back(absolutePose);
    if (relativePose != nullptr)
        this->WindowRelPoses.push_back(*relativePose);

    int maxWindow = std::max(2, this->Vop2elParams.SlidingWindowSize);
    while (static_cast<int>(this->WindowPoses.size()) > maxWindow)
    {
        this->WindowPoses.pop_front();
        if (!this->WindowRelPoses.empty())
            this->WindowRelPoses.pop_front();
        ++this->WindowStartIndex;
    }
}

//-------------------------------------------------------------------------------------------
void Vop2elAlgorithm::MaybeOptimizeSlidingWindow()
{
    if (!this->Vop2elParams.EnableSlidingWindow)
        return;

    if (this->Vop2elParams.SlidingWindowSize < 2)
        return;

    int totalPoses = 0;
    int totalRel = 0;
    {
        std::lock_guard<std::mutex> lock(this->PosesMutex);
        totalPoses = static_cast<int>(this->WindowPoses.size());
        totalRel = static_cast<int>(this->WindowRelPoses.size());
    }

    if (totalPoses < 2 || totalRel < 1)
        return;

    int windowSize = std::min(this->Vop2elParams.SlidingWindowSize, totalPoses);
    int startIndex = 0;

    std::vector<Eigen::Affine3d> windowPoses;
    std::vector<Eigen::Affine3d> windowRel;
    {
        std::lock_guard<std::mutex> lock(this->PosesMutex);
        startIndex = this->WindowStartIndex;
        windowPoses.assign(this->WindowPoses.begin(), this->WindowPoses.end());
        windowRel.assign(this->WindowRelPoses.begin(), this->WindowRelPoses.end());
    }

    if (windowRel.size() < 1)
        return;

    if (!this->Vop2elParams.SlidingWindowBackground)
    {
        this->OptimizeSlidingWindowThread(startIndex, windowSize, windowPoses, windowRel);
        return;
    }

    if (this->OptimizationInProgress.exchange(true))
        return;

    if (this->OptimizerThread.joinable())
        this->OptimizerThread.join();

    this->OptimizerThread = std::thread(&Vop2elAlgorithm::OptimizeSlidingWindowThread,
                                        this, startIndex, windowSize, windowPoses, windowRel);
}

//-------------------------------------------------------------------------------------------
void Vop2elAlgorithm::OptimizeSlidingWindowThread(int startIndex, int windowSize,
                                                const std::vector<Eigen::Affine3d>& windowPoses,
                                                const std::vector<Eigen::Affine3d>& windowRelPoses)
{
    struct PoseParam
    {
        double q[4];
        double t[3];
    };

    std::vector<PoseParam> params(windowSize);
    for (int i = 0; i < windowSize; ++i)
    {
        Eigen::Quaterniond q(windowPoses[i].rotation());
        params[i].q[0] = q.w();
        params[i].q[1] = q.x();
        params[i].q[2] = q.y();
        params[i].q[3] = q.z();
        params[i].t[0] = windowPoses[i].translation().x();
        params[i].t[1] = windowPoses[i].translation().y();
        params[i].t[2] = windowPoses[i].translation().z();
    }

    ceres::Problem problem;
    for (int i = 0; i < windowSize - 1; ++i)
    {
        Eigen::Quaterniond q_meas(windowRelPoses[i].rotation());
        Eigen::Vector3d t_meas = windowRelPoses[i].translation();

        ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<RelativePoseCost, 6, 4, 3, 4, 3>(
            new RelativePoseCost(q_meas, t_meas));
        ceres::LossFunction* loss = new ceres::HuberLoss(1.0);
        problem.AddResidualBlock(cost, loss, params[i].q, params[i].t, params[i + 1].q, params[i + 1].t);
    }

    ceres::LocalParameterization* quat_param = new ceres::EigenQuaternionParameterization();
    for (int i = 0; i < windowSize; ++i)
        problem.SetParameterization(params[i].q, quat_param);

    // Fix first pose in window to anchor.
    problem.SetParameterBlockConstant(params[0].q);
    problem.SetParameterBlockConstant(params[0].t);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 30;
    options.num_threads = GetSolverNumThreads();

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::vector<Eigen::Affine3d> optimized(windowSize);
    for (int i = 0; i < windowSize; ++i)
    {
        Eigen::Quaterniond q(params[i].q[0], params[i].q[1], params[i].q[2], params[i].q[3]);
        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        pose.linear() = q.normalized().toRotationMatrix();
        pose.translation() = Eigen::Vector3d(params[i].t[0], params[i].t[1], params[i].t[2]);
        optimized[i] = pose;
    }

    {
        std::lock_guard<std::mutex> lock(this->PosesMutex);
        if (static_cast<int>(this->AbsolutePoses.size()) >= startIndex + windowSize)
        {
            for (int i = 0; i < windowSize; ++i)
                this->AbsolutePoses[startIndex + i] = optimized[i];
        }
        if (this->WindowStartIndex == startIndex && static_cast<int>(this->WindowPoses.size()) == windowSize)
        {
            for (int i = 0; i < windowSize; ++i)
                this->WindowPoses[i] = optimized[i];
        }
    }

    this->OptimizationInProgress.store(false);
}
}
