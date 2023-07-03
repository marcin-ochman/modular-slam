#ifndef MSLAM_PROJECTION_HPP_
#define MSLAM_PROJECTION_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/camera_parameters.hpp"
#include "modular_slam/slam3d_types.hpp"

namespace mslam
{

Vector2 projectOnImage(const Vector3& point, const CameraParameters& cameraParams);
Vector3 toCameraCoordinates(const Vector3& point, const slam3d::SensorState& sensorPose);
slam3d::SensorState toCameraCoordinateSystemProjection(const slam3d::SensorState& state);
const slam3d::SensorState& toWorldCoordinateSystemProjection(const slam3d::SensorState& state);
Vector2 projectOnImage(const Vector3& point, const CameraParameters& cameraParams,
                       const slam3d::SensorState& sensorPose);
bool isVisibleInFrame(const Vector3& point, const CameraParameters& cameraParams, const Size& resolution);
Vector3 toGlobalCoordinates(const Vector3& point, const slam3d::SensorState& sensorPose);
bool isVisibleInFrame(const Vector3& point, const slam3d::SensorState& cameraPose, const CameraParameters& cameraParams,
                      const Size& resolution);
} // namespace mslam

#endif // MSLAM_PROJECTION_HPP_
