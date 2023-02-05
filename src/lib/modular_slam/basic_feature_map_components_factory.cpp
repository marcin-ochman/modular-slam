#include "modular_slam/basic_feature_map_components_factory.hpp"
#include "modular_slam/slam3d_types.hpp"

namespace mslam
{

std::shared_ptr<Landmark<slam3d::LandmarkState>> BasicFeatureMapComponentsFactory::createLandmark()
{
    static Id currentLandmarkId = 0;
    auto landmark = std::make_shared<Landmark<slam3d::LandmarkState>>();

    landmark->id = currentLandmarkId++;

    return landmark;
}

std::shared_ptr<Keyframe<slam3d::SensorState>> BasicFeatureMapComponentsFactory::createKeyframe()
{
    return std::make_shared<Keyframe<slam3d::SensorState>>();
}

} // namespace mslam
