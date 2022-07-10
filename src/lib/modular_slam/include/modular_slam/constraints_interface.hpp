#ifndef MSLAM_CONSTRAINTS_INTERFACE_HPP_
#define MSLAM_CONSTRAINTS_INTERFACE_HPP_

#include "modular_slam/observation.hpp"

namespace mslam
{
template <typename SensorStateType, typename LandmarkStateType>
class ConstraintsInterface
{
  public:
    virtual void addLandmarkConstraint(std::shared_ptr<Observation<SensorStateType, LandmarkStateType>>) = 0;
    virtual void addKeyframeConstraint(std::shared_ptr<Observation<SensorStateType, LandmarkStateType>>) = 0;

    virtual void allLandmarks() = 0;
    virtual void allKeyframes() = 0;
};

} // namespace mslam

#endif // MSLAM_CONSTRAINTS_INTERFACE_HPP_
