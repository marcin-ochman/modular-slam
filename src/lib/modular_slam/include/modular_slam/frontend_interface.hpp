#ifndef MSLAM_FRONTEND_INTERFACE_HPP_
#define MSLAM_FRONTEND_INTERFACE_HPP_

#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/slam_component.hpp"
#include <memory>

namespace mslam
{

template <typename SensorDataType>
struct LoopDetection
{
    std::shared_ptr<Keyframe<SensorDataType>> firstKeyframe;
    std::shared_ptr<Keyframe<SensorDataType>> secondKeyframe;
    // std::vector<> commonLandmarks;
};

template <typename SensorStateType, typename LandmarkStateType>
struct FrontendOutput
{
    std::optional<LoopDetection<SensorStateType>> loopDetected;
    std::vector<LandmarkObservation<SensorStateType, LandmarkStateType>> landmarkObservations;
    std::shared_ptr<Keyframe<SensorStateType>> newKeyframe;
    std::vector<std::shared_ptr<Landmark<LandmarkStateType>>> newLandmarks;

    SensorStateType sensorState;
};

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType>
class FrontendInterface : public SlamComponent
{
  public:
    using FrontendOutputType = FrontendOutput<SensorStateType, LandmarkStateType>;

    virtual std::shared_ptr<FrontendOutputType> processSensorData(const SensorDataType& sensorData) = 0;
    virtual ~FrontendInterface() = default;
};
} // namespace mslam

#endif // MSLAM_FRONTEND_INTERFACE_HPP_
