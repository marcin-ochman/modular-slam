#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cstdint>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <future>
#include <vector>

namespace mslam
{

using Id = std::uint64_t;

using Vector2d = Eigen::Vector2f;
using Vector3d = Eigen::Vector3f;
using Quaternion = Eigen::Quaternionf;

struct RgbFrame
{
    std::vector<std::uint8_t> rgbData;
};

struct DepthFrame
{
    std::vector<std::uint16_t> depthData;
};

struct RgbdFrame : RgbFrame, DepthFrame
{
};

struct RgbdiFrame : RgbdFrame
{
};

template <typename PositionType, typename OrientationType>
struct State
{
    PositionType position;
    OrientationType orientation;
};

template <typename StateType>
struct Landmark
{
    Id id;
    StateType state;
};

template <typename StateType>
struct Keyframe
{
    Id id;
    StateType state;
};

template <typename KeyframeType, typename LandmarkStateType>
struct Observation
{
    std::shared_ptr<KeyframeType> firstKeyframe;
    std::shared_ptr<KeyframeType> secondKeyframe;
    std::shared_ptr<Landmark<LandmarkStateType>> landmark;
};

namespace slam2d
{
using State = mslam::State<Vector2d, float>;
using Keyframe = mslam::Keyframe<State>;
} // namespace slam2d

namespace slam3d
{
using State = mslam::State<Vector3d, Quaternion>;
using Keyframe = mslam::Keyframe<State>;
} // namespace slam3d

} // namespace mslam

#endif // TYPES_HPP_
