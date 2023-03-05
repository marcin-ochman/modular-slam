#ifndef MSLAM_FRONTEND_INTERFACE_HPP_
#define MSLAM_FRONTEND_INTERFACE_HPP_

#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/slam_component.hpp"
#include <any>
#include <memory>

namespace mslam
{

template <typename SensorDataType>
struct LoopDetection
{
    std::shared_ptr<Keyframe<SensorDataType>> firstKeyframe;
    std::shared_ptr<Keyframe<SensorDataType>> secondKeyframe;
};

template <typename SensorStateType, typename LandmarkStateType>
struct FrontendOutput
{
    std::optional<LoopDetection<SensorStateType>> loopDetected;
    std::vector<LandmarkObservation<LandmarkStateType>> landmarkObservations;
    std::shared_ptr<Keyframe<SensorStateType>> newKeyframe;
    std::vector<std::shared_ptr<Landmark<LandmarkStateType>>> newLandmarks;
    SensorStateType sensorState;
    SensorStateType slamState;
};

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType>
class FrontendInterface : public SlamComponent
{
  public:
    using FrontendOutputType = FrontendOutput<SensorStateType, LandmarkStateType>;

    virtual FrontendOutputType processSensorData(std::shared_ptr<SensorDataType> sensorData) = 0;
    ~FrontendInterface() override = default;
};
} // namespace mslam

#endif // MSLAM_FRONTEND_INTERFACE_HPP_
