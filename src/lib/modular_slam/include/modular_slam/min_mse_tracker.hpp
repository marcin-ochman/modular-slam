#ifndef MSLAM_MIN_MSE_TRACKER_HPP_
#define MSLAM_MIN_MSE_TRACKER_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/slam3d_types.hpp"
#include "modular_slam/tracker.hpp"

namespace mslam
{

class MinMseTracker : public Tracker<slam3d::SensorState, Vector3>
{
  public:
    std::optional<slam3d::SensorState> track(const std::vector<std::shared_ptr<Landmark<Vector3>>>& landmarks,
                                             const std::vector<Vector3>& sensorPoints) override;
};

} // namespace mslam

#endif // MSLAM_MIN_MSE_TRACKER_HPP_
