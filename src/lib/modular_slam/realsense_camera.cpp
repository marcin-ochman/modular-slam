#include "modular_slam/realsense_camera.hpp"
#include "modular_slam/modular_slam.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <algorithm>
#include <cassert>
#include <cstdint>

#include <iostream>

namespace mslam
{

Eigen::Matrix3f getProjectionMatrix(const CameraParameters& cameraParameters)
{
    const auto& focal = cameraParameters.focal;
    const auto& principal = cameraParameters.principalPoint;

    Eigen::Matrix3f projection;
    projection << focal.x(), 0.0f, principal.x(), 0.0f, focal.y(), principal.y(), 0.0f, 0.0f, 1.0f;

    return projection;
}

Eigen::Matrix3f getInverseProjectionMatrix(const CameraParameters& cameraParameters)
{
    auto projectionMatrix = getProjectionMatrix(cameraParameters);

    return projectionMatrix.inverse();
}

bool RealSenseCamera::init()
{
    pipe.start();

    return true;
}

bool RealSenseCamera::fetch()
{
    frames = pipe.wait_for_frames();
    frames = align_to_color.process(frames);
    const auto intrinsics = frames.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    auto newRgbdFrame = std::make_shared<RgbdFrame>();

    newRgbdFrame->depth.cameraParameters.focal = {intrinsics.fx, intrinsics.fy};
    newRgbdFrame->depth.cameraParameters.principalPoint = {intrinsics.ppx, intrinsics.ppy};
    newRgbdFrame->depth.cameraParameters.factor = 0.001f;

    rs2::video_frame rgb_frame = frames.first(RS2_STREAM_COLOR);
    rs2::depth_frame depth_frame = frames.first(RS2_STREAM_DEPTH);

    assert(rgb_frame.get_width() == depth_frame.get_width());
    assert(rgb_frame.get_height() == depth_frame.get_height());

    auto rgbMemorySize =
        static_cast<std::size_t>(rgb_frame.get_height() * rgb_frame.get_width() * rgb_frame.get_bytes_per_pixel());
    auto depthMemorySize = static_cast<std::size_t>(depth_frame.get_height() * depth_frame.get_width() *
                                                    depth_frame.get_bytes_per_pixel());

    newRgbdFrame->rgb.data.resize(rgbMemorySize);
    newRgbdFrame->depth.data.resize(depthMemorySize);
    newRgbdFrame->rgb.size.height = rgb_frame.get_height();
    newRgbdFrame->rgb.size.width = rgb_frame.get_width();
    newRgbdFrame->depth.size.height = depth_frame.get_height();
    newRgbdFrame->depth.size.width = depth_frame.get_width();

    std::copy_n(reinterpret_cast<const uint16_t*>(depth_frame.get_data()), depth_frame.get_data_size(),
                newRgbdFrame->depth.data.begin());

    cv::Mat rgbMatFrame{rgb_frame.get_height(), rgb_frame.get_width(), CV_8UC3,
                        const_cast<void*>(rgb_frame.get_data())},
        bgrFrame{rgb_frame.get_height(), rgb_frame.get_width(), CV_8UC3, newRgbdFrame->rgb.data.data()};

    cv::cvtColor(rgbMatFrame, bgrFrame, cv::COLOR_RGBA2BGR);

    rgbd = std::move(newRgbdFrame);

    return true;
}

std::shared_ptr<RgbdFrame> RealSenseCamera::recentData() const
{
    return rgbd;
}

} // namespace mslam
