#ifndef REALSENSE_CAMERA_HPP_
#define REALSENSE_CAMERA_HPP_

#include "modular_slam/camera.hpp"
#include <librealsense2/rs.hpp>

namespace mslam
{

class RealSenseCamera : public Camera<RgbdFrame>
{

  public:
    virtual bool init() override;
    virtual bool fetch() override;
    virtual std::shared_ptr<RgbdFrame> recentData() const override;

  private:
    // rs2::frameset data;
    // rs2::align align_to_color(RS2_STREAM_COLOR);
};
} // namespace mslam

#endif /* REALSENSE_CAMERA_HPP_ */
