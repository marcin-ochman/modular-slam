#ifndef MSLAM_DEPTH_FRAME_HPP_
#define MSLAM_DEPTH_FRAME_HPP_

#include <cstddef>
#include <cstdint>
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

inline std::uint16_t getDepth(const DepthFrame& depthFrame, const Eigen::Vector2i& imgPoint)
{
    std::size_t index = depthFrame.size.width * imgPoint.y() + imgPoint.x();

    return depthFrame.data[index];
}

inline bool isDepthValid(std::uint16_t depth)
{
    return depth > 0;
}

} // namespace mslam

#endif // MSLAM_DEPTH_FRAME_HPP_
