#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <cstdint>
#include <vector>

namespace mslam
{
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

template <typename T>
class Slam
{
  public:
    bool init();
    bool run();
};

} // namespace mslam

#endif // TYPES_HPP_
