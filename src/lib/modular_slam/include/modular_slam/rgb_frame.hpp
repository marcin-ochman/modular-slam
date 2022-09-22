#ifndef MSLAM_RGB_FRAME_HPP_
#define MSLAM_RGB_FRAME_HPP_

#include <cstdint>
#include <vector>

#include "modular_slam/basic_types.hpp"
#include "modular_slam/grayscale_frame.hpp"

namespace mslam
{
struct RgbFrame
{
    std::vector<std::uint8_t> data;
    Size size;
};

GrayScaleFrame toGrayScale(const RgbFrame& rgb);

} // namespace mslam

#endif // MSLAM_RGB_FRAME_HPP_
