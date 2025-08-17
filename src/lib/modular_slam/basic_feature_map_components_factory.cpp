#include "modular_slam/map/basic_feature_map_components_factory.hpp"

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
