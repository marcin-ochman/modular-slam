#ifndef MSLAM_MODULAR_SLAM_HPP_
#define MSLAM_MODULAR_SLAM_HPP_

// Here put only public headers

#include "modular_slam/slam_builder.hpp"
#include "modular_slam/types/keyframe.hpp"
#include "modular_slam/types/state.hpp"

namespace mslam
{

namespace slam2d
{
using State = mslam::State<Vector2, float>;
using Keyframe = mslam::Keyframe<State>;
} // namespace slam2d
} // namespace mslam

#endif /* MSLAM_MODULAR_SLAM_HPP_ */
