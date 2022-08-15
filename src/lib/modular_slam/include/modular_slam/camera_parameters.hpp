#ifndef MSLAM_CAMERA_PARAMETERS_HPP_
#define MSLAM_CAMERA_PARAMETERS_HPP_

#include "modular_slam/basic_types.hpp"
namespace mslam
{
struct CameraParameters
{
    Vector2d principalPoint;
    Vector2d focal;
};

Eigen::Matrix3f getProjectionMatrix(const CameraParameters& cameraParameters);
Eigen::Matrix3f getInverseProjectionMatrix(const CameraParameters& cameraParameters);

} // namespace mslam

#endif // MSLAM_CAMERA_PARAMETERS_HPP_
