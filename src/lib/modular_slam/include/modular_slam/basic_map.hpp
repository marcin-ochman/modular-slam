#ifndef BASIC_MAP_HPP_
#define BASIC_MAP_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/feature_interface.hpp"
#include "modular_slam/map_interface.hpp"
#include "modular_slam/rgbd_slam_types.hpp"
#include "modular_slam/slam3d_types.hpp"
#include <unordered_set>

#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index_container.hpp>

namespace mslam
{

class BasicMap : public IMap<slam3d::SensorState, slam3d::LandmarkState, rgbd::RgbdKeypoint>
{
  public:
    BasicMap();
    void update(
        const FrontendOutput<slam3d::SensorState, slam3d::LandmarkState, rgbd::RgbdKeypoint>& frontendOutput) override;

    void update(const BackendOutputType& frontendOutput) override;
    void visit(IMapVisitor<slam3d::SensorState, slam3d::LandmarkState, rgbd::RgbdKeypoint>& visitor,
               const MapVisitingParams& params = {}) override;

    std::shared_ptr<slam3d::Keyframe> getKeyframeById(Id keyframeId);
    bool hasKeyframe(std::shared_ptr<slam3d::Keyframe> keyframe) const;
    void addKeyframe(std::shared_ptr<slam3d::Keyframe> keyframe);
    void removeKeyframe(std::shared_ptr<slam3d::Keyframe> keyframe);
    void
    addObservation(const LandmarkKeyframeObservation<slam3d::SensorState, slam3d::LandmarkState, rgbd::RgbdKeypoint>&);

    bool hasLandmark(std::shared_ptr<slam3d::Landmark> landmark) const;
    void addLandmark(std::shared_ptr<slam3d::Landmark> landmark);
    void removeLandmark(std::shared_ptr<slam3d::Landmark> landmark);

  private:
    void updateCovisibility(
        std::shared_ptr<Keyframe<slam3d::SensorState>> newKeyframe,
        const std::vector<LandmarkObservation<slam3d::LandmarkState, rgbd::RgbdKeypoint>>& landmarkObservations);

    void visitKeyframeNeighbours(IMapVisitor<slam3d::SensorState, slam3d::LandmarkState, rgbd::RgbdKeypoint>& visitor,
                                 const GraphBasedParams& graph, std::shared_ptr<slam3d::Keyframe> refKeyframe);

    void
    visitLandmarksOfNeighbours(IMapVisitor<slam3d::SensorState, slam3d::LandmarkState, rgbd::RgbdKeypoint>& visitor,
                               const GraphBasedParams& graph, std::shared_ptr<slam3d::Keyframe> refKeyframe);

    std::unordered_set<std::shared_ptr<slam3d::Keyframe>>
    getNeighbourKeyframes(const GraphBasedParams& graph, std::shared_ptr<slam3d::Keyframe> refKeyframe);

    std::unordered_set<std::shared_ptr<slam3d::Keyframe>> keyframes;
    std::unordered_set<std::shared_ptr<slam3d::Landmark>> landmarks;

    using ObservationType = LandmarkKeyframeObservation<slam3d::SensorState, slam3d::LandmarkState, rgbd::RgbdKeypoint>;
    using ObservationsMultiIndexContainer = boost::multi_index::multi_index_container<
        ObservationType, boost::multi_index::indexed_by<
                             boost::multi_index::hashed_non_unique<boost::multi_index::member<
                                 ObservationType, std::shared_ptr<slam3d::Keyframe>, &ObservationType::keyframe>>,
                             boost::multi_index::hashed_non_unique<boost::multi_index::member<
                                 ObservationType, std::shared_ptr<slam3d::Landmark>, &ObservationType::landmark>>>>;

    ObservationsMultiIndexContainer indexedObservations;
    std::unordered_map<std::shared_ptr<slam3d::Keyframe>, std::set<std::shared_ptr<slam3d::Keyframe>>> neighbours;
};
} // namespace mslam

#endif // BASIC_MAP_HPP_
