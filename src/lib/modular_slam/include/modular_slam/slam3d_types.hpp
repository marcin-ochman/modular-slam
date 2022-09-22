#ifndef MSLAM_SLAM3D_HPP_
#define MSLAM_SLAM3D_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/keyframe.hpp"
#include "modular_slam/state.hpp"
#include <Eigen/src/Core/Matrix.h>

namespace mslam
{

namespace slam3d
{
using SensorState = mslam::State<Vector3, Quaternion>;
using Keyframe = mslam::Keyframe<SensorState>;
} // namespace slam3d

} // namespace mslam

#endif // MSLAM_SLAM3D_HPP_
