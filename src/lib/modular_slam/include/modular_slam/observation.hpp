#ifndef MSLAM_OBSERVATION_HPP_
#define MSLAM_OBSERVATION_HPP_

#include <memory>

#include "modular_slam/feature_interface.hpp"
#include "modular_slam/landmark.hpp"

namespace mslam
{

template <typename SensorStateType, typename LandmarkStateType, typename ObservationType = Keypoint>
struct LandmarkKeyframeObservation
{
    std::shared_ptr<Keyframe<SensorStateType>> keyframe;
    std::shared_ptr<Landmark<LandmarkStateType>> landmark;
    ObservationType observation;
};

template <typename LandmarkStateType, typename ObservationType>
struct LandmarkObservation
{
    std::shared_ptr<Landmark<LandmarkStateType>> landmark;
    ObservationType observation;
};

template <typename SensorStateType, typename LandmarkStateType, typename KeyframeConstraintType = SensorStateType>
struct LoopDetected
{
    std::shared_ptr<Keyframe<SensorStateType>> firstKeyframe;
    std::shared_ptr<Keyframe<SensorStateType>> secondKeyframe;
};

// template <typename KeyframeType, typename LandmarkStateType>
// struct Observation
// {
//     std::shared_ptr<KeyframeType> keyframe;
//     std::shared_ptr<Landmark<LandmarkStateType>> landmark;
// };

} // namespace mslam

#endif // MSLAM_OBSERVATION_INTERFACE_HPP_
