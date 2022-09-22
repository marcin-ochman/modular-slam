#ifndef MSLAM_FRONTEND_INTERFACE_HPP_
#define MSLAM_FRONTEND_INTERFACE_HPP_

#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/slam_component.hpp"
#include <memory>

namespace mslam
{

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType>
class FrontendInterface : public SlamComponent
{
  public:
    using Constraints = ConstraintsInterface<SensorStateType, LandmarkStateType>;

    FrontendInterface() = default;

    FrontendInterface(const FrontendInterface<SensorDataType, SensorStateType, LandmarkStateType>&) = delete;
    FrontendInterface(FrontendInterface<SensorDataType, SensorStateType, LandmarkStateType>&&) = delete;
    FrontendInterface<SensorDataType, SensorStateType, LandmarkStateType>&
    operator=(const FrontendInterface<SensorDataType, SensorStateType, LandmarkStateType>&) = delete;
    FrontendInterface<SensorDataType, SensorStateType, LandmarkStateType>&
    operator=(const FrontendInterface<SensorDataType, SensorStateType, LandmarkStateType>&&) = delete;

    virtual std::shared_ptr<Constraints> prepareConstraints(const SensorDataType& sensorData) = 0;
    virtual bool update(const Constraints& /*constraints*/) { return true; }

    virtual ~FrontendInterface() {}
};

} // namespace mslam

#endif // MSLAM_FRONTEND_INTERFACE_HPP_
