#ifndef MSLAM_DEPTH_FRAME_HPP_
#define MSLAM_DEPTH_FRAME_HPP_

#include <cstdint>
#include <vector>

#include "modular_slam/basic_types.hpp"

namespace mslam
{
struct DepthFrame
{
    std::vector<std::uint16_t> data;
    Size size;
};

} // namespace mslam

#endif // MSLAM_DEPTH_FRAME_HPP_
