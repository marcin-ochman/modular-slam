#ifndef MSLAM_FEATURE_FRONTEND_HPP_
#define MSLAM_FEATURE_FRONTEND_HPP_

#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/slam_component.hpp"

namespace mslam
{
template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType>
class FeatureFrontend : public FrontendInterface<SensorDataType, SensorStateType, LandmarkStateType>
{
  public:
    using Constraints = typename FrontendInterface<SensorDataType, SensorStateType, LandmarkStateType>::Constraints;

    virtual std::shared_ptr<Constraints> prepareConstraints(const SensorDataType& sensorData) = 0;
};
} // namespace mslam

#endif // MSLAM_FEATURE_FRONTEND_HPP_
