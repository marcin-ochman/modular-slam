#ifndef MSLAM_RGBD_FEATURE_FRONTEND_HPP_
#define MSLAM_RGBD_FEATURE_FRONTEND_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/feature_interface.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/keyframe.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/loop_detection.hpp"
#include "modular_slam/map_interface.hpp"
#include "modular_slam/observation.hpp"
#include "modular_slam/orb_feature.hpp"
#include "modular_slam/pnp.hpp"
#include "modular_slam/relocalizer.hpp"
#include "modular_slam/rgb_frame.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/slam3d_types.hpp"
#include "modular_slam/slam_component.hpp"

#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index_container.hpp>

#include <Eigen/src/Core/Matrix.h>
#include <cstddef>
#include <memory>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include <modular_slam/rgbd_slam_types.hpp>
#include <unordered_map>
#include <unordered_set>

namespace mslam
{

class RgbdFeatureFrontend : public FrontendInterface<RgbdFrame, slam3d::SensorState, Vector3, rgbd::RgbdKeypoint>
{
  public:
    using FrontendOutputType =
        typename FrontendInterface<RgbdFrame, slam3d::SensorState, Vector3, rgbd::RgbdKeypoint>::FrontendOutputType;

    RgbdFeatureFrontend(
        std::shared_ptr<IPnpAlgorithm<slam3d::SensorState, Vector3>> tracker,
        std::shared_ptr<IFeatureDetector<RgbFrame, float, 32>> detector,
        std::shared_ptr<IFeatureMatcher<float, 32>> matcher,
        std::shared_ptr<IMapComponentsFactory<slam3d::SensorState, slam3d::LandmarkState>> mapComponentsFactory,
        std::shared_ptr<IMap<rgbd::SensorState, rgbd::LandmarkState, rgbd::RgbdKeypoint>> map);

    FrontendOutputType processSensorData(std::shared_ptr<RgbdFrame> sensorData) override;

    void update(const BackendOutputType& backendOutput) override;
    bool init() override;

  protected:
    [[nodiscard]] std::shared_ptr<rgbd::Keyframe> findBetterReferenceKeyframe(const slam3d::SensorState& currentPose,
                                                                              const RgbdFrame& frame) const;
    [[nodiscard]] bool isBetterReferenceKeyframeNeeded(const std::size_t keyframeLandmarksCount) const;
    [[nodiscard]] bool isNewKeyframeRequired(const std::size_t matchedLandmarks) const;
    [[nodiscard]] bool hasInitialKeyframe() const { return referenceKeyframe != nullptr; }
    [[nodiscard]] bool isLoopClosureNeeded() const;
    FrontendOutputType initFirstKeyframe(const RgbdFrame& sensorData,
                                         const std::vector<KeypointDescriptor<float, 32>>& keypoints);

    std::shared_ptr<rgbd::Keyframe> relocalize(std::vector<KeypointDescriptor<float, 32>>& keypoints);

    RgbdFeatureFrontend::FrontendOutputType track(const RgbdFrame& sensorData,
                                                  std::vector<KeypointDescriptor<float, 32>>& keypoints);

    std::shared_ptr<rgbd::Keyframe> addKeyframe(const RgbdFrame& sensorData, const slam3d::SensorState& pose,
                                                const std::vector<KeypointDescriptor<float, 32>>& keypoints);
    [[nodiscard]] std::size_t minMatchedPoints() const;
    [[nodiscard]] std::vector<std::shared_ptr<rgbd::Landmark>>
    findVisibleLocalLandmarks(const slam3d::SensorState& pose, const RgbdFrame& sensorData) const;

    struct LandmarkMatches
    {
        std::vector<KeypointDescriptor<float, 32>> keypointsForNewLandmarks;
        std::unordered_map<std::shared_ptr<rgbd::Landmark>, KeypointDescriptor<float, 32>> matchedLandmarks;
    };

    [[nodiscard]] LandmarkMatches
    findKeypointsForLandmarks(const std::vector<KeypointDescriptor<float, 32>>& keypoints,
                              const boost::span<std::shared_ptr<rgbd::Landmark>> landmarks,
                              const slam3d::SensorState& pose, const RgbdFrame& sensorData) const;

    std::shared_ptr<rgbd::Landmark> addLandmark(std::shared_ptr<rgbd::Keyframe>&, const Vector3& state,
                                                const KeypointDescriptor<float, 32>& keypoint);

    void bindKeypointToLandmark(const KeypointDescriptor<float, 32>& keypointWithDescriptor,
                                const Vector3& coordinatesInCameraFrame, std::shared_ptr<rgbd::Landmark> landmark,
                                std::shared_ptr<rgbd::Keyframe> keyframe);

    void removeObservation(const rgbd::KeyframeLandmarkObservation& observation);

    void addNewLandmarks(const std::vector<KeypointDescriptor<float, 32>>& newLandmarkKeypoints,
                         std::shared_ptr<rgbd::Keyframe> newKeyframe, const RgbdFrame& sensorData,
                         FrontendOutputType& output);

    void addLandmarkObservation(std::shared_ptr<rgbd::Keyframe> keyframe, const rgbd::LandmarkObservation& observation);

    void updateVisibleLandmarks(
        const std::unordered_map<std::shared_ptr<rgbd::Landmark>, KeypointDescriptor<float, 32>>& matchedLandmarks,
        const std::shared_ptr<rgbd::Keyframe>& keyframe, const RgbdFrame& sensorData);

    std::vector<KeypointLandmarkMatch<Vector3>>
    matchLandmarks(const std::vector<KeypointDescriptor<float, 32>>& keypoints,
                   std::shared_ptr<rgbd::Keyframe> keyframe);

    std::pair<std::vector<KeypointDescriptor<float, 32>>, std::vector<std::shared_ptr<rgbd::Landmark>>>
    getLandmarksWithKeypoints(std::shared_ptr<rgbd::Keyframe> keyframe);
    void closeLoop(std::shared_ptr<Keyframe<slam3d::SensorState>>& loopCandidate);

  private:
    // algorithms
    std::shared_ptr<IFeatureDetector<RgbFrame, float, 32>> detector;
    std::shared_ptr<IFeatureMatcher<float, 32>> matcher;
    std::shared_ptr<IPnpAlgorithm<slam3d::SensorState, Vector3>> pnpAlgorithm;
    std::shared_ptr<IMapComponentsFactory<slam3d::SensorState, slam3d::LandmarkState>> mapComponentsFactory;
    std::shared_ptr<ILoopDetector<slam3d::SensorState>> loopDetector;
    std::shared_ptr<IRelocalizer<slam3d::SensorState, float, 32>> relocalizer;

    // local map/graph data
    std::shared_ptr<rgbd::Keyframe> referenceKeyframe;

    // TODO: this is redundant to data contained by map
    // ***************************************************************************************
    //*
    using ObservationsMultiIndexContainer = boost::multi_index::multi_index_container<
        rgbd::Observation,
        boost::multi_index::indexed_by<
            boost::multi_index::hashed_non_unique<boost::multi_index::member<
                rgbd::Observation, std::shared_ptr<slam3d::Keyframe>, &rgbd::Observation::keyframe>>,
            boost::multi_index::hashed_non_unique<boost::multi_index::member<
                rgbd::Observation, std::shared_ptr<slam3d::Landmark>, &rgbd::Observation::landmark>>>>;

    ObservationsMultiIndexContainer allObservations;

    // ****************************************************************************************************************************************//*
    //
    slam3d::SensorState currentPose;
    std::shared_ptr<IMap<rgbd::SensorState, rgbd::LandmarkState, rgbd::RgbdKeypoint>> map;
};
} // namespace mslam

#endif // MSLAM_RGBD_FEATURE_FRONTEND_HPP_
