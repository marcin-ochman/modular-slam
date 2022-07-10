#ifndef MSLAM_OBSERVATION_HPP_
#define MSLAM_OBSERVATION_HPP_

#include <memory>

#include "modular_slam/landmark.hpp"

namespace mslam
{

template <typename KeyframeType, typename LandmarkStateType>
struct Observation
{
    std::shared_ptr<KeyframeType> firstKeyframe;
    std::shared_ptr<KeyframeType> secondKeyframe;
    std::shared_ptr<Landmark<LandmarkStateType>> landmark;
};

} // namespace mslam

#endif // MSLAM_OBSERVATION_INTERFACE_HPP_
