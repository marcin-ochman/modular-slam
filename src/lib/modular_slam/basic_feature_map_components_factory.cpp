#include "modular_slam/basic_feature_map_components_factory.hpp"
#include "modular_slam/slam3d_types.hpp"

namespace mslam
{

std::shared_ptr<Landmark<slam3d::LandmarkState>> BasicFeatureMapComponentsFactory::createLandmark()
{
    auto landmark = std::make_shared<Landmark<slam3d::LandmarkState>>();

    landmark->id = currentLandmarkId++;

    return landmark;
}

std::shared_ptr<Keyframe<slam3d::SensorState>> BasicFeatureMapComponentsFactory::createKeyframe()
{
    auto keyframe = std::make_shared<Keyframe<slam3d::SensorState>>();

    keyframe->id = currentKeyframeId++;
    return keyframe;
}

} // namespace mslam
