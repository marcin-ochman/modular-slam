#ifndef MSLAM_CONSTRAINTS_INTERFACE_HPP_
#define MSLAM_CONSTRAINTS_INTERFACE_HPP_

#include "modular_slam/keyframe.hpp"
#include "modular_slam/observation.hpp"

namespace mslam
{
template <typename SensorStateType, typename LandmarkStateType>
class ConstraintsInterface
{
  public:
    virtual void addLandmarkConstraint(const Observation<SensorStateType, LandmarkStateType>&) = 0;
    virtual void addKeyframeConstraint(const Keyframe<SensorStateType>& firstKeyframe,
                                       const Keyframe<SensorStateType>& secondKeyframe) = 0;
};

} // namespace mslam

#endif // MSLAM_CONSTRAINTS_INTERFACE_HPP_
