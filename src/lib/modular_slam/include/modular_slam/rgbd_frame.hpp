#ifndef MSLAM_RGBD_FRAME_HPP_
#define MSLAM_RGBD_FRAME_HPP_

#include "modular_slam/camera.hpp"
#include "modular_slam/depth_frame.hpp"
#include "modular_slam/rgb_frame.hpp"
#include "modular_slam/timestamp.hpp"

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
