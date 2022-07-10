#ifndef MSLAM_FRONTEND_INTERFACE_HPP_
#define MSLAM_FRONTEND_INTERFACE_HPP_

#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/slam_component.hpp"

namespace mslam
{

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType>
class FrontendInterface : public SlamComponent
{
  public:
    virtual std::shared_ptr<ConstraintsInterface<SensorStateType, LandmarkStateType>>
    prepareConstraints(std::shared_ptr<SensorDataType> sensorData) = 0;
};

} // namespace mslam

#endif // MSLAM_FRONTEND_INTERFACE_HPP_
