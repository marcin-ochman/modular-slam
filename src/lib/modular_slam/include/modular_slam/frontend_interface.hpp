#ifndef MSLAM_FRONTEND_INTERFACE_HPP_
#define MSLAM_FRONTEND_INTERFACE_HPP_

#include "modular_slam/backend_output.hpp"
#include "modular_slam/frontend_output.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/slam_component.hpp"
#include <any>
#include <memory>

namespace mslam
{

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType, typename ObservationType>
class FrontendInterface : public SlamComponent
{
  public:
    using FrontendOutputType = FrontendOutput<SensorStateType, LandmarkStateType, ObservationType>;
    using BackendOutputType = BackendOutput<SensorStateType, LandmarkStateType, ObservationType>;

    virtual FrontendOutputType processSensorData(std::shared_ptr<SensorDataType> sensorData) = 0;
    ~FrontendInterface() override = default;

    virtual void update(const BackendOutputType& sensorData) = 0;
};
} // namespace mslam

#endif // MSLAM_FRONTEND_INTERFACE_HPP_
