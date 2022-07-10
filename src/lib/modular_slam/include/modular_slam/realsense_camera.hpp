#ifndef REALSENSE_CAMERA_HPP_
#define REALSENSE_CAMERA_HPP_

#include "modular_slam/camera.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include <librealsense2/h/rs_sensor.h>
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
    rs2::pipeline pipe;
    rs2::frameset frames;

    std::shared_ptr<RgbdFrame> rgbd;
    rs2::align align_to_color{RS2_STREAM_COLOR};
};
} // namespace mslam

#endif /* REALSENSE_CAMERA_HPP_ */
