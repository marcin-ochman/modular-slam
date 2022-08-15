#include "modular_slam/camera_parameters.hpp"

namespace mslam
{

Eigen::Matrix3f getProjectionMatrix(const CameraParameters& cameraParameters)
{
    const auto& focal = cameraParameters.focal;
    const auto& principal = cameraParameters.principalPoint;

    Eigen::Matrix3f projection;
    projection << focal.x(), 0.0f, principal.x(), 0.0f, focal.y(), principal.y(), 0.0f, 0.0f, 1.0f;

    return projection;
}

Eigen::Matrix3f getInverseProjectionMatrix(const CameraParameters& cameraParameters)
{
    auto projectionMatrix = getProjectionMatrix(cameraParameters);

    return projectionMatrix.inverse();
}

} // namespace mslam
