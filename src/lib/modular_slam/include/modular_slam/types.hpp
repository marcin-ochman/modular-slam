#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <cstdint>
#include <future>
#include <vector>

namespace mslam
{

using Id = std::uint64_t;

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

template <typename KeyframeType, typename LandmarkType>
struct Observation
{
    std::shared_ptr<KeyframeType> firstKeyframe;
    std::shared_ptr<KeyframeType> secondKeyframe;
};

} // namespace mslam

#endif // TYPES_HPP_
