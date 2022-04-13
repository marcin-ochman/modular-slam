#ifndef MODULAR_SLAM_HPP_
#define MODULAR_SLAM_HPP_

#include "modular_slam/loop_detection.hpp"

namespace mslam
{

using RgbFrame = int; // TODO: change it
struct DepthFrame
{
};

struct RgbdFrame
{
    RgbFrame rgb;
};

} // namespace mslam

#endif /* MODULAR_SLAM_HPP_ */
