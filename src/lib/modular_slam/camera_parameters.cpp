#include "modular_slam/camera_parameters.hpp"

namespace mslam
{

Matrix3 getProjectionMatrix(const CameraParameters& cameraParameters)
{
    const auto& focal = cameraParameters.focal;
    const auto& principal = cameraParameters.principalPoint;

    Matrix3 projection;
    projection << focal.x(), 0.0f, principal.x(), 0.0f, focal.y(), principal.y(), 0.0f, 0.0f, 1.0f;

    return projection;
}

Matrix3 getInverseProjectionMatrix(const CameraParameters& cameraParameters)
{
    auto projectionMatrix = getProjectionMatrix(cameraParameters);

    return projectionMatrix.inverse();
}

} // namespace mslam
