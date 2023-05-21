#ifndef MSLAM_GRAYSCALE_HPP_
#define MSLAM_GRAYSCALE_HPP_

#include "modular_slam/basic_types.hpp"

namespace mslam
{
struct GrayScaleFrame
{
    std::vector<std::uint8_t> data;
    Size size;
};
} // namespace mslam

#endif // MSLAM_GRAYSCALE_HPP_
