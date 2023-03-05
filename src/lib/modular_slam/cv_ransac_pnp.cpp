#include "modular_slam/cv_ransac_pnp.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace mslam
{
std::optional<OpenCvRansacPnp::PnpResult>
OpenCvRansacPnp::solvePnp(const std::vector<std::shared_ptr<Landmark<Vector3>>>& landmarks,
                          const std::vector<Vector2>& sensorPoints, const slam3d::SensorState& initial)
{

    cv::Mat objectPoints(static_cast<int>(landmarks.size()), 3, CV_32F);

    for(std::size_t i = 0; i < landmarks.size(); ++i)
    {
        const auto& landmark = landmarks[i];
        auto* rowPtr = objectPoints.ptr<float>(static_cast<int>(i));

        rowPtr[0] = landmark->state.x();
        rowPtr[1] = landmark->state.y();
        rowPtr[2] = landmark->state.z();
    }

    cv::Mat imagePoints(static_cast<int>(sensorPoints.size()), 2, CV_32F);
    for(std::size_t i = 0; i < sensorPoints.size(); ++i)
    {
        const auto& point = sensorPoints[i];
        auto* rowPtr = imagePoints.ptr<float>(static_cast<int>(i));

        rowPtr[0] = point.x();
        rowPtr[1] = point.y();
    }

    cv::Mat cvRotation, cvTranslation;
    cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << cameraParams.focal.x(), 0, cameraParams.principalPoint.x(), 0,
                            cameraParams.focal.y(), cameraParams.principalPoint.y(), 0, 0, 1);

    std::vector<int> inliers;
    const auto success = cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, cv::noArray(), cvRotation,
                                            cvTranslation, true, 100, 8.0, 0.99, inliers);

    if(!success)
        return std::nullopt;

    const auto angle = cv::norm(cvRotation);
    Matrix3 rotation =
        Eigen::AngleAxisd(angle,
                          Vector3(cvRotation.at<double>(0), cvRotation.at<double>(1), cvRotation.at<double>(2)) / angle)
            .toRotationMatrix();

    const auto translation =
        Vector3(cvTranslation.at<double>(0), cvTranslation.at<double>(1), cvTranslation.at<double>(2));

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = translation;

    const auto invTransform = transform.inverse();

    PnpResult result;
    result.pose.orientation = invTransform.block<3, 3>(0, 0);
    result.pose.position = invTransform.block<3, 1>(0, 3);

    result.inliers.resize(landmarks.size(), false);

    std::for_each(std::cbegin(inliers), std::cend(inliers), [&result](int index) { result.inliers[index] = false; });

    return result;
}

} // namespace mslam
