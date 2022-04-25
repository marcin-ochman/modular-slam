#ifndef MODULAR_SLAM_HPP_
#define MODULAR_SLAM_HPP_

#include "modular_slam/loop_detection.hpp"
#include <cstdint>
#include <vector>

namespace mslam
{

struct RgbFrame
{
    std::vector<uint8_t> rgbData;
};

struct DepthFrame
{
    std::vector<uint16_t> depthData;
};

struct RgbdFrame : RgbFrame, DepthFrame
{
};

// cv::Mat toCvMat();

} // namespace mslam

#endif /* MODULAR_SLAM_HPP_ */
