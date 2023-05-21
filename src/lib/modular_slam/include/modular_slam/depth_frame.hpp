#ifndef MSLAM_DEPTH_FRAME_HPP_
#define MSLAM_DEPTH_FRAME_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "modular_slam/basic_types.hpp"
#include "modular_slam/camera.hpp"
#include "modular_slam/camera_parameters.hpp"

namespace mslam
{
struct DepthFrame
{
    std::vector<std::uint16_t> data;
    Size size;
    CameraParameters cameraParameters;
};

inline float getDepth(const DepthFrame& depthFrame, const Eigen::Vector2i& imgPoint)
{
    const auto index = static_cast<std::size_t>(depthFrame.size.width * imgPoint.y() + imgPoint.x());

    return depthFrame.data[index] * depthFrame.cameraParameters.factor;
}

inline bool isDepthValid(const float depth)
{
    return depth > std::numeric_limits<float>::epsilon();
}

} // namespace mslam

#endif // MSLAM_DEPTH_FRAME_HPP_
