#ifndef MSLAM_TRACKER_HPP_
#define MSLAM_TRACKER_HPP_

#include "modular_slam/landmark.hpp"

#include <memory>
#include <optional>

namespace mslam
{

template <typename SensorStateType, typename LandmarkStateType, typename PointType = LandmarkStateType>
class Tracker
{
  public:
    virtual std::optional<SensorStateType>
    track(const std::vector<std::shared_ptr<Landmark<LandmarkStateType>>>& landmarks,
          const std::vector<PointType>& sensorPoints) = 0;
};

} // namespace mslam

#endif // MSLAM_TRACKER_HPP_
