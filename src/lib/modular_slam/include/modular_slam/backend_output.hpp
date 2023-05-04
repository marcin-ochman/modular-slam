#ifndef MSLAM_BACKEND_OUTPUT_HPP_
#define MSLAM_BACKEND_OUTPUT_HPP_

#include "modular_slam/keyframe.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/observation.hpp"

#include <memory>
#include <unordered_set>

namespace mslam
{
template <typename SensorStateType, typename LandmarkStateType, typename ObservationType>
struct BackendOutput
{
    using LandmarkType = Landmark<LandmarkStateType>;
    using KeyframeType = Keyframe<SensorStateType>;

    std::unordered_set<std::shared_ptr<LandmarkType>> updatedLandmarks;
    std::unordered_set<std::shared_ptr<KeyframeType>> updatedKeyframes;
    std::vector<LandmarkKeyframeObservation<SensorStateType, LandmarkStateType, ObservationType>> outlierObservations;
};

} // namespace mslam

#endif // MSLAM_BACKEND_OUTPUT_HPP_
