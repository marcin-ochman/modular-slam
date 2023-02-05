#ifndef BASIC_FEATURE_MAP_COMPONENTS_FACTORY_HPP__
#define BASIC_FEATURE_MAP_COMPONENTS_FACTORY_HPP__

#include "modular_slam/map_interface.hpp"
#include "modular_slam/slam3d_types.hpp"

namespace mslam
{
class BasicFeatureMapComponentsFactory : public IFeatureMapComponentsFactory<slam3d::SensorState, slam3d::LandmarkState>
{
  public:
    std::shared_ptr<Landmark<slam3d::LandmarkState>> createLandmark() override;
    std::shared_ptr<Keyframe<slam3d::SensorState>> createKeyframe() override;
};
} // namespace mslam

#endif // BASIC_FEATURE_MAP_COMPONENTS_FACTORY_HPP__
