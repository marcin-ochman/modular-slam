#ifndef MSLAM_MAP_INTERFACE_HPP_
#define MSLAM_MAP_INTERFACE_HPP_

#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/keyframe.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/slam_component.hpp"

namespace mslam
{

template <typename SensorStateType, typename LandmarkStateType>
class IFeatureMapComponentsFactory
{
  public:
    virtual std::shared_ptr<Landmark<LandmarkStateType>> createLandmark() = 0;
    virtual std::shared_ptr<Keyframe<SensorStateType>> createKeyframe() = 0;
    virtual ~IFeatureMapComponentsFactory() = default;
};

template <typename SensorStateType, typename LandmarkStateType>
class MapInterface : public SlamComponent
{
  public:
    using Constraints = ConstraintsInterface<SensorStateType, LandmarkStateType>;
    virtual void update(const std::shared_ptr<Constraints> constraints) = 0;
};

} // namespace mslam

#endif // MAP_HPP_
