#ifndef MSLAM_FEATURE_FRONTEND_HPP_
#define MSLAM_FEATURE_FRONTEND_HPP_

#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/slam_component.hpp"

namespace mslam
{
template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType, typename ObservationType>
class FeatureFrontend : public FrontendInterface<SensorDataType, SensorStateType, LandmarkStateType, ObservationType>
{
  public:
};
} // namespace mslam

#endif // MSLAM_FEATURE_FRONTEND_HPP_
