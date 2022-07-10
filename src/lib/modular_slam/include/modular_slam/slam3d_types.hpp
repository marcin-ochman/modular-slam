#ifndef MSLAM_SLAM3D_HPP_
#define MSLAM_SLAM3D_HPP_

#include "modular_slam/keyframe.hpp"
#include "modular_slam/state.hpp"

namespace mslam
{

namespace slam3d
{
using State = mslam::State<Vector3d, Quaternion>;
using Keyframe = mslam::Keyframe<State>;
} // namespace slam3d

} // namespace mslam

#endif // MSLAM_SLAM3D_HPP_
