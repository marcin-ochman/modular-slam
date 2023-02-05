#include "modular_slam/cv_ransac_pnp.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>

namespace mslam
{
std::optional<slam3d::SensorState>
mslam::OpenCvRansacPnp::solvePnp(const std::vector<std::shared_ptr<Landmark<Vector3>>>& landmarks,
                                 const std::vector<Vector2>& sensorPoints, const slam3d::SensorState& initial)
{

    cv::Mat objectPoints(landmarks.size(), 3, CV_32F);

    for(auto i = 0; i < landmarks.size(); ++i)
    {
        const auto& landmark = landmarks[i];
        auto* rowPtr = objectPoints.ptr<float>(i);

        rowPtr[0] = landmark->state.x();
        rowPtr[1] = landmark->state.y();
        rowPtr[2] = landmark->state.z();
    }

    cv::Mat imagePoints(static_cast<int>(sensorPoints.size()), 2, CV_32F);
    for(auto i = 0; i < sensorPoints.size(); ++i)
    {
        const auto& point = sensorPoints[i];
        auto* rowPtr = imagePoints.ptr<float>(i);

        rowPtr[0] = point.x();
        rowPtr[1] = point.y();
    }

    cv::Mat cvRotation, cvTranslation;
    cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << cameraParams.focal.x(), 0, cameraParams.principalPoint.x(), 0,
                            cameraParams.focal.y(), cameraParams.principalPoint.y(), 0, 0, 1);

    const auto success =
        cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, cv::Mat(), cvRotation, cvTranslation);

    if(!success)
        return std::nullopt;

    slam3d::SensorState result;

    const auto angle = cv::norm(cvRotation);
    Eigen::Matrix3f rotation =
        Eigen::AngleAxisd(
            angle,
            Eigen::Vector3d(cvRotation.at<double>(0), cvRotation.at<double>(1), cvRotation.at<double>(2)) / angle)
            .toRotationMatrix()
            .cast<float>();

    const auto translation =
        Eigen::Vector3d(cvTranslation.at<double>(0), cvTranslation.at<double>(1), cvTranslation.at<double>(2));

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = translation.cast<float>();

    const auto invTransform = transform.inverse();
    result.orientation = invTransform.block<3, 3>(0, 0);
    result.position = invTransform.block<3, 1>(0, 3);

    return result;
}

} // namespace mslam
