#ifndef MSLAM_SLAM3D_HPP_
#define MSLAM_SLAM3D_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/keyframe.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/state.hpp"

namespace mslam
{

namespace slam3d
{
using SensorState = mslam::State<Vector3, Quaternion>;
using LandmarkState = Vector3;
using Keyframe = mslam::Keyframe<SensorState>;
using Landmark = mslam::Landmark<Vector3>;
} // namespace slam3d

} // namespace mslam

#endif // MSLAM_SLAM3D_HPP_
