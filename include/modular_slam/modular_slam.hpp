#ifndef MODULAR_SLAM_HPP_
#define MODULAR_SLAM_HPP_

#include "modular_slam/loop_detection.hpp"
#include "modular_slam/odometry.hpp"
#include "modular_slam/types.hpp"

namespace mslam
{

class ModularSlam
{
  public:
    bool init();
};

template <typename DataProviderType>
std::unique_ptr<ModularSlam> make_mslam();

} // namespace mslam

#endif /* MODULAR_SLAM_HPP_ */
