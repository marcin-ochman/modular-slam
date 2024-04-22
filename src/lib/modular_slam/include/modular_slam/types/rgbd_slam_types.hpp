#ifndef RGBD_SLAM_TYPES_HPP_
#define RGBD_SLAM_TYPES_HPP_

#include "modular_slam/frontend/feature/feature_interface.hpp"
#include "modular_slam/types/slam3d_types.hpp"
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

template <typename DescriptorType = std::uint8_t, int Length = 32>
struct RgbdKeypointDescriptor
{
    RgbdKeypoint keypoint;
    std::array<DescriptorType, Length> descriptor;
};

using RgbdOrbKeypointDescriptor = RgbdKeypointDescriptor<std::uint8_t, 32>;
using LandmarkObservation = LandmarkObservation<LandmarkState, RgbdOrbKeypointDescriptor>;
using KeyframeLandmarkObservation = LandmarkKeyframeObservation<SensorState, LandmarkState, RgbdOrbKeypointDescriptor>;

struct Observation
{
    RgbdOrbKeypointDescriptor keypoint;
    std::shared_ptr<rgbd::Landmark> landmark;
    std::shared_ptr<rgbd::Keyframe> keyframe;
};

} // namespace mslam::rgbd

#endif // RGBD_SLAM_TYPES_HPP_
