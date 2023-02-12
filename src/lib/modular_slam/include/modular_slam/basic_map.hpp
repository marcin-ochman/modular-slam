#ifndef BASIC_MAP_HPP_
#define BASIC_MAP_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/map_interface.hpp"
#include "modular_slam/slam3d_types.hpp"
#include <unordered_set>

namespace mslam
{
class BasicMap : public IMap<slam3d::SensorState, slam3d::LandmarkState>
{
  public:
    BasicMap();
    void update(const std::shared_ptr<Constraints> constraints) override;
    void visit(IMapVisitor<slam3d::SensorState, slam3d::LandmarkState>& visitor,
               const MapVisitingParams& params = {}) override;

    bool hasKeyframe(std::shared_ptr<slam3d::Keyframe> keyframe) const;
    void addKeyframe(std::shared_ptr<slam3d::Keyframe> keyframe);
    void removeKeyframe(std::shared_ptr<slam3d::Keyframe> keyframe);

    bool hasLandmark(std::shared_ptr<slam3d::Landmark> landmark) const;
    void addLandmark(std::shared_ptr<slam3d::Landmark> landmark);
    void removeLandmark(std::shared_ptr<slam3d::Landmark> landmark);

  private:
    class ConstraintVisitor : public IConstraintVisitor<slam3d::SensorState, slam3d::LandmarkState>
    {
      public:
        explicit ConstraintVisitor(BasicMap& map);
        void
        visit(const LandmarkObservationConstraint<slam3d::SensorState, slam3d::LandmarkState>& constraint) override;
        void visit(const KeyframeConstraint<slam3d::SensorState, slam3d::LandmarkState>& constraint) override;

      private:
        BasicMap& map;
    };

    ConstraintVisitor visitor;
    std::unordered_set<std::shared_ptr<slam3d::Keyframe>> keyframes;
    std::unordered_set<std::shared_ptr<slam3d::Landmark>> landmarks;
};
} // namespace mslam

#endif // BASIC_MAP_HPP_
