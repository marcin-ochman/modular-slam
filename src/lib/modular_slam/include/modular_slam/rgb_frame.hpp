#ifndef MSLAM_RGB_FRAME_HPP_
#define MSLAM_RGB_FRAME_HPP_

#include <cstdint>
#include <vector>

namespace mslam
{
struct RgbFrame
{
    std::vector<std::uint8_t> rgbData;
};

} // namespace mslam

#endif // MSLAM_RGB_FRAME_HPP_
