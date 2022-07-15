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
    virtual void addLandmarkConstraint(std::shared_ptr<Observation<SensorStateType, LandmarkStateType>>) = 0;
    virtual void addKeyframeConstraint(std::shared_ptr<Keyframe<SensorStateType>> firstKeyframe,
                                       std::shared_ptr<Keyframe<SensorStateType>> secondKeyframe) = 0;
};

} // namespace mslam

#endif // MSLAM_CONSTRAINTS_INTERFACE_HPP_
