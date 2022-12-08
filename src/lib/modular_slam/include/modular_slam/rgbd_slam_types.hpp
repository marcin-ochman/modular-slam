#ifndef RGBD_SLAM_TYPES_HPP_
#define RGBD_SLAM_TYPES_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/feature_interface.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/slam3d_types.hpp"
#include <modular_slam/keyframe.hpp>

namespace mslam::rgbd
{
using SensorState = mslam::slam3d::SensorState;
using Keyframe = Keyframe<SensorState>;
using Landmark = Landmark<Vector3>;
using FeatureInterface = FeatureInterface<Vector2, Vector3>;
} // namespace mslam::rgbd

#endif // RGBD_SLAM_TYPES_HPP_
