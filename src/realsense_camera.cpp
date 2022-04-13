#include "modular_slam/realsense_camera.hpp"

namespace mslam
{

bool RealSenseCamera::init()
{

    return true;
}

bool RealSenseCamera::fetch()
{
    return true;
}

  std::shared_ptr<RgbdFrame> RealSenseCamera::recentData() const
{
    return nullptr;
}

} // namespace mslam
