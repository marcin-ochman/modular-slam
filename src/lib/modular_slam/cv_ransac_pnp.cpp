#include "modular_slam/cv_ransac_pnp.hpp"
#include "modular_slam/basic_types.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <spdlog/spdlog.h>

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

        rowPtr[0] = static_cast<float>(landmark->state.x());
        rowPtr[1] = static_cast<float>(landmark->state.y());
        rowPtr[2] = static_cast<float>(landmark->state.z());
    }

    cv::Mat imagePoints(static_cast<int>(sensorPoints.size()), 2, CV_32F);
    for(std::size_t i = 0; i < sensorPoints.size(); ++i)
    {
        const auto& point = sensorPoints[i];
        auto* const rowPtr = imagePoints.ptr<float>(static_cast<int>(i));

        rowPtr[0] = static_cast<float>(point.x());
        rowPtr[1] = static_cast<float>(point.y());
    }

    Eigen::AngleAxisd angleAxis;
    angleAxis = initial.orientation;

    cv::Mat cvRotation = ((cv::Mat_<double>(3, 1) << angleAxis.axis().x() * angleAxis.angle(),
                           angleAxis.axis().y() * angleAxis.angle(), angleAxis.axis().z() * angleAxis.angle()));
    cv::Mat cvTranslation =
        (cv::Mat_<double>(3, 1) << initial.position.x(), initial.position.y(), initial.position.z());

    cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << cameraParams.focal.x(), 0, cameraParams.principalPoint.x(), 0,
                            cameraParams.focal.y(), cameraParams.principalPoint.y(), 0, 0, 1);

    std::vector<int> inliers;
    const auto success = cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, cv::noArray(), cvRotation,
                                            cvTranslation, false, 100, 8.0, 0.99, inliers);

    if(!success)
    {
        spdlog::error("Didnt find pnp solution");
        return std::nullopt;
    }

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
    std::for_each(std::cbegin(inliers), std::cend(inliers),
                  [&result](int index) { result.inliers[static_cast<std::size_t>(index)] = true; });

    return result;
}

} // namespace mslam
