#ifndef MSLAM_RGBD_FRAME_HPP_
#define MSLAM_RGBD_FRAME_HPP_

#include "modular_slam/types/depth_frame.hpp"
#include "modular_slam/types/rgb_frame.hpp"
#include "modular_slam/types/timestamp.hpp"

#include <cstdint>
#include <vector>

namespace mslam
{
struct RgbdFrame
{
    Timestamp timestamp;
    RgbFrame rgb;
    DepthFrame depth;
    // CameraParameters cameraParameters;
};

} // namespace mslam

#endif // MSLAM_RGBD_FRAME_HPP_
