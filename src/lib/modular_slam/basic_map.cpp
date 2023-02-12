#include "modular_slam/basic_map.hpp"
#include "modular_slam/constraints_interface.hpp"

namespace mslam
{

BasicMap::BasicMap() : visitor(*this) {}

void BasicMap::update(const std::shared_ptr<Constraints> constraints)
{
    constraints->visitConstraints(visitor);
}

void BasicMap::visit(IMapVisitor<slam3d::SensorState, slam3d::LandmarkState>& visitor, const MapVisitingParams& params)
{
    for(auto& keyframe : keyframes)
    {
        visitor.visit(keyframe);
    }

    for(auto& landmark : landmarks)
    {
        visitor.visit(landmark);
    }
}

bool BasicMap::hasKeyframe(std::shared_ptr<slam3d::Keyframe> keyframe) const
{
    return keyframes.count(keyframe) > 0;
}

void BasicMap::addKeyframe(std::shared_ptr<slam3d::Keyframe> keyframe)
{
    keyframes.insert(keyframe);
}

void BasicMap::removeKeyframe(std::shared_ptr<slam3d::Keyframe> keyframe)
{
    keyframes.erase(keyframe);
}

bool BasicMap::hasLandmark(std::shared_ptr<slam3d::Landmark> landmark) const
{
    return landmarks.count(landmark) > 0;
}

void BasicMap::addLandmark(std::shared_ptr<slam3d::Landmark> landmark)
{
    landmarks.insert(landmark);
}

void BasicMap::removeLandmark(std::shared_ptr<slam3d::Landmark> landmark)
{
    landmarks.erase(landmark);
}

BasicMap::ConstraintVisitor::ConstraintVisitor(BasicMap& newMap) : map(newMap) {}

void BasicMap::ConstraintVisitor::visit(
    const LandmarkObservationConstraint<slam3d::SensorState, slam3d::LandmarkState>& constraint)
{
    map.addKeyframe(constraint.keyframe);
    map.addLandmark(constraint.landmark);
}

void BasicMap::ConstraintVisitor::visit(
    const KeyframeConstraint<slam3d::SensorState, slam3d::LandmarkState>& constraint)
{
    map.addKeyframe(constraint.firstKeyframe);
    map.addKeyframe(constraint.secondKeyframe);
}
} // namespace mslam
