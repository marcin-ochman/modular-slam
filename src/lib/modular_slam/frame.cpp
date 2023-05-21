#include "modular_slam/rgb_frame.hpp"

namespace mslam
{

GrayScaleFrame toGrayScale(const RgbFrame& rgb)
{
    GrayScaleFrame grayFrame;

    constexpr auto elementsPerPixel = 3;
    grayFrame.data.reserve(rgb.data.size() / elementsPerPixel);

    for(std::size_t i = 0; i < rgb.data.size(); i += elementsPerPixel)
    {
        const auto r = rgb.data[i];
        const auto g = rgb.data[i + 1];
        const auto b = rgb.data[i + 2];
        std::uint8_t value = static_cast<std::uint8_t>(std::min(
            static_cast<float>(std::numeric_limits<std::uint8_t>::max()), 0.299f * r + 0.587f * g + 0.114f * b));

        grayFrame.data.push_back(value);
    }

    grayFrame.size = rgb.size;

    return grayFrame;
}

} // namespace mslam
