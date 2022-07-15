#ifndef MSLAM_DEPTH_FRAME_HPP_
#define MSLAM_DEPTH_FRAME_HPP_

#include <cstdint>
#include <vector>

namespace mslam
{
struct DepthFrame
{
    std::vector<std::uint16_t> data;
};

} // namespace mslam

#endif // MSLAM_DEPTH_FRAME_HPP_
