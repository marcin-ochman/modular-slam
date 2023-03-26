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

template <typename SensorStateType, typename LandmarkStateType, typename ObservationType>
struct FrontendOutput
{
    std::optional<LoopDetection<SensorStateType>> loopDetected;
    std::vector<LandmarkObservation<LandmarkStateType, ObservationType>> landmarkObservations;
    std::shared_ptr<Keyframe<SensorStateType>> newKeyframe;
    std::vector<std::shared_ptr<Landmark<LandmarkStateType>>> newLandmarks;

    // TODO: update with traits!
    // TODO 2: it would be more efficient to split keypoint / descriptor data
    SensorStateType sensorState;
    // SensorStateType slamState;
};

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType, typename ObservationType>
class FrontendInterface : public SlamComponent
{
  public:
    using FrontendOutputType = FrontendOutput<SensorStateType, LandmarkStateType, ObservationType>;

    virtual FrontendOutputType processSensorData(std::shared_ptr<SensorDataType> sensorData) = 0;
    ~FrontendInterface() override = default;
};
} // namespace mslam

#endif // MSLAM_FRONTEND_INTERFACE_HPP_
