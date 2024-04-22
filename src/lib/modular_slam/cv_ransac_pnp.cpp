#include "modular_slam/cv_ransac_pnp.hpp"
#include "modular_slam/projection.hpp"

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

    assert(landmarks.size() == sensorPoints.size());
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

    auto initialProjection = toCameraCoordinateSystemProjection(initial);

    Eigen::AngleAxisd angleAxis;
    angleAxis = initialProjection.orientation;

    cv::Mat cvRotation = ((cv::Mat_<double>(3, 1) << angleAxis.axis().x() * angleAxis.angle(),
                           angleAxis.axis().y() * angleAxis.angle(), angleAxis.axis().z() * angleAxis.angle()));
    cv::Mat cvTranslation = (cv::Mat_<double>(3, 1) << initialProjection.position.x(), initialProjection.position.y(),
                             initialProjection.position.z());

    cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << cameraParams.focal.x(), 0, cameraParams.principalPoint.x(), 0,
                            cameraParams.focal.y(), cameraParams.principalPoint.y(), 0, 0, 1);

    std::vector<int> inliers;
    const auto success = cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, cv::noArray(), cvRotation,
                                            cvTranslation, true, 100, 5.0, 0.99, inliers);

    if(!success)
    {
        spdlog::error("Didnt find pnp solution");
        return std::nullopt;
    }

    const auto angle = cv::norm(cvRotation);
    Eigen::AngleAxisd angleAxisResult =
        Eigen::AngleAxisd(angle,
                          Vector3(cvRotation.at<double>(0), cvRotation.at<double>(1), cvRotation.at<double>(2)) / angle)
            .inverse();

    spdlog::info("Inliers {} / {} ", inliers.size(), landmarks.size());

    const auto translation =
        Vector3(cvTranslation.at<double>(0), cvTranslation.at<double>(1), cvTranslation.at<double>(2));

    PnpResult result;
    result.pose.orientation = angleAxisResult;
    result.pose.position = -(angleAxisResult * translation);

    result.inliers.resize(landmarks.size(), false);
    std::for_each(std::cbegin(inliers), std::cend(inliers),
                  [&result](int index) { result.inliers[static_cast<std::size_t>(index)] = true; });

    return result;
}

} // namespace mslam
