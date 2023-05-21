#ifndef RGBD_SLAM_TYPES_HPP_
#define RGBD_SLAM_TYPES_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/feature_interface.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/keyframe.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/slam3d_types.hpp"
#include <cstdint>

namespace mslam::rgbd
{
using SensorState = mslam::slam3d::SensorState;
using LandmarkState = Vector3;
using Keyframe = Keyframe<SensorState>;
using Landmark = Landmark<LandmarkState>;

struct RgbdKeypoint
{
    Keypoint keypoint;
    double depth;
};

template <typename DescriptorType, int Length = 32>
struct RgbdKeypointDescriptor
{
    RgbdKeypoint keypoint;
    std::array<DescriptorType, Length> descriptor;
};

struct Observation
{
    RgbdKeypointDescriptor<float, 32> keypoint;
    std::shared_ptr<rgbd::Landmark> landmark;
    std::shared_ptr<rgbd::Keyframe> keyframe;
};

using LandmarkObservation = LandmarkObservation<LandmarkState, RgbdKeypoint>;
using KeyframeLandmarkObservation = LandmarkKeyframeObservation<SensorState, LandmarkState, RgbdKeypoint>;
} // namespace mslam::rgbd

#endif // RGBD_SLAM_TYPES_HPP_
