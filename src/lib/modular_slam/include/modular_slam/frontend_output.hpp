#ifndef MSLAM_FRONTEND_OUTPUT_HPP_
#define MSLAM_FRONTEND_OUTPUT_HPP_

#include <memory>
#include <optional>
#include <vector>

#include "modular_slam/keyframe.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/observation.hpp"

namespace mslam
{

enum class SlamState
{
    Initializing,
    Tracking,
    Lost
};

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
    SensorStateType pose;
    SlamState slamState;
};

} // namespace mslam

#endif // MSLAM_FRONTEND_OUTPUT_HPP_
