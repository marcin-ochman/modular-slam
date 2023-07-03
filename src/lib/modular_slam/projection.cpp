#include "modular_slam/projection.hpp"

namespace mslam
{

Vector2 projectOnImage(const Vector3& point, const CameraParameters& cameraParams)
{
    const auto z = point.z();

    return point.block<2, 1>(0, 0).array() / z * cameraParams.focal.array() + cameraParams.principalPoint.array();
}

Vector3 toCameraCoordinates(const Vector3& point, const slam3d::SensorState& sensorPose)
{
    const auto inverse = sensorPose.orientation.inverse();
    return inverse * point - inverse * sensorPose.position;
}

slam3d::SensorState toCameraCoordinateSystemProjection(const slam3d::SensorState& state)
{
    const auto inverse = state.orientation.inverse();

    slam3d::SensorState invState;
    invState.orientation = inverse;
    invState.position = -(inverse * state.position);

    return invState;
}

const slam3d::SensorState& toWorldCoordinateSystemProjection(const slam3d::SensorState& state)
{
    return state;
}

Vector2 projectOnImage(const Vector3& point, const CameraParameters& cameraParams,
                       const slam3d::SensorState& sensorPose)
{
    const auto pointCameraCoordinates = toCameraCoordinates(point, sensorPose);
    return projectOnImage(pointCameraCoordinates, cameraParams);
}

bool isVisibleInFrame(const Vector3& point, const CameraParameters& cameraParams, const Size& resolution)
{
    const auto imagePoint = projectOnImage(point, cameraParams);
    const auto inWidth = imagePoint.x() >= 0 && imagePoint.x() < static_cast<float>(resolution.width);
    const auto inHeight = imagePoint.y() >= 0 && imagePoint.y() < static_cast<float>(resolution.height);

    return inWidth && inHeight && point.z() > 0.0f;
}

Vector3 toGlobalCoordinates(const Vector3& point, const slam3d::SensorState& sensorPose)
{
    return sensorPose.orientation * point + sensorPose.position;
}

bool isVisibleInFrame(const Vector3& point, const slam3d::SensorState& cameraPose, const CameraParameters& cameraParams,
                      const Size& resolution)
{
    auto pointInCameraCoordinates = toCameraCoordinates(point, cameraPose);

    return isVisibleInFrame(pointInCameraCoordinates, cameraParams, resolution);
}
} // namespace mslam
