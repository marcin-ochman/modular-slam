#ifndef MSLAM_RGBD_FEATURE_FRONTEND_HPP_
#define MSLAM_RGBD_FEATURE_FRONTEND_HPP_

#include "modular_slam/frontend/feature/feature_interface.hpp"
#include "modular_slam/frontend/feature_frontend.hpp"
#include "modular_slam/loop_detection.hpp"
#include "modular_slam/map/map_interface.hpp"
#include "modular_slam/observation.hpp"
#include "modular_slam/orb_feature.hpp"
#include "modular_slam/pnp.hpp"
#include "modular_slam/relocalizer.hpp"
#include "modular_slam/slam_component.hpp"
#include "modular_slam/types/rgb_frame.hpp"
#include "modular_slam/types/rgbd_frame.hpp"
#include "modular_slam/types/slam3d_types.hpp"

#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index_container.hpp>

#include <Eigen/src/Core/Matrix.h>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include <modular_slam/types/rgbd_slam_types.hpp>
#include <unordered_map>
#include <unordered_set>

namespace mslam
{

class RgbdFeatureFrontend
    : public FeatureFrontend<RgbdFrame, slam3d::SensorState, Vector3, rgbd::RgbdOrbKeypointDescriptor>
{
  public:
    using FrontendOutputType = typename FrontendInterface<RgbdFrame, slam3d::SensorState, Vector3,
                                                          rgbd::RgbdOrbKeypointDescriptor>::FrontendOutputType;

    RgbdFeatureFrontend(
        std::shared_ptr<IPnpAlgorithm<slam3d::SensorState, Vector3>> tracker,
        std::shared_ptr<IFeatureDetector<RgbFrame, std::uint8_t, 32>> detector,
        std::shared_ptr<IFeatureMatcher<std::uint8_t, 32>> matcher,
        std::shared_ptr<IMapComponentsFactory<slam3d::SensorState, slam3d::LandmarkState>> mapComponentsFactory,
        std::shared_ptr<IMap<rgbd::SensorState, rgbd::LandmarkState, rgbd::RgbdOrbKeypointDescriptor>> map);

    FrontendOutputType processSensorData(std::shared_ptr<RgbdFrame> sensorData) override;

    void update(const BackendOutputType& backendOutput) override;
    bool init() override;

  protected:
    [[nodiscard]] std::shared_ptr<rgbd::Keyframe> findBetterReferenceKeyframe(FrontendOutputType& result,
                                                                              const RgbdFrame& frame) const;
    [[nodiscard]] bool isBetterReferenceKeyframeNeeded(const std::size_t keyframeLandmarksCount) const;
    [[nodiscard]] bool isNewKeyframeRequired(const std::size_t matchedLandmarks) const;
    [[nodiscard]] bool hasInitialKeyframe() const { return referenceKeyframe != nullptr; }
    [[nodiscard]] bool isLoopClosureNeeded() const;
    FrontendOutputType initFirstKeyframe(const RgbdFrame& sensorData,
                                         const std::vector<KeypointDescriptor<std::uint8_t, 32>>& keypoints);

    std::shared_ptr<rgbd::Keyframe> relocalize(std::vector<KeypointDescriptor<std::uint8_t, 32>>& keypoints);

    RgbdFeatureFrontend::FrontendOutputType track(const RgbdFrame& sensorData,
                                                  std::vector<KeypointDescriptor<std::uint8_t, 32>>& keypoints);

    std::shared_ptr<rgbd::Keyframe> addKeyframe(const RgbdFrame& sensorData, const slam3d::SensorState& pose,
                                                const std::vector<KeypointDescriptor<std::uint8_t, 32>>& keypoints);
    [[nodiscard]] std::size_t minMatchedPoints() const;

    struct LandmarkMatches
    {
        std::vector<KeypointDescriptor<std::uint8_t, 32>> keypointsForNewLandmarks;
        std::unordered_map<std::shared_ptr<rgbd::Landmark>, KeypointDescriptor<std::uint8_t, 32>> matchedLandmarks;
    };

    std::shared_ptr<rgbd::Landmark> addLandmark(std::shared_ptr<rgbd::Keyframe>&, const Vector3& state,
                                                const KeypointDescriptor<std::uint8_t, 32>& keypoint);

    void removeObservation(const rgbd::KeyframeLandmarkObservation& observation);

    void addNewLandmarks(const std::vector<KeypointDescriptor<std::uint8_t, 32>>& newLandmarkKeypoints,
                         std::shared_ptr<rgbd::Keyframe> newKeyframe, const RgbdFrame& sensorData,
                         FrontendOutputType& output);

    std::pair<std::vector<KeypointDescriptor<std::uint8_t, 32>>, std::vector<std::shared_ptr<rgbd::Landmark>>>
    matchLandmarks(const std::vector<KeypointDescriptor<std::uint8_t, 32>>& keypoints,
                   std::shared_ptr<rgbd::Keyframe> keyframe);

    std::pair<std::vector<KeypointDescriptor<std::uint8_t, 32>>, std::vector<std::shared_ptr<rgbd::Landmark>>>
    getLandmarksWithKeypoints(std::shared_ptr<rgbd::Keyframe> keyframe);
    void closeLoop(std::shared_ptr<Keyframe<slam3d::SensorState>>& loopCandidate);

  private:
    std::shared_ptr<IFeatureDetector<RgbFrame, std::uint8_t, 32>> detector;
    std::shared_ptr<IFeatureMatcher<std::uint8_t, 32>> matcher;
    std::shared_ptr<IPnpAlgorithm<slam3d::SensorState, Vector3>> pnpAlgorithm;
    std::shared_ptr<IMapComponentsFactory<slam3d::SensorState, slam3d::LandmarkState>> mapComponentsFactory;
    std::shared_ptr<ILoopDetector<slam3d::SensorState>> loopDetector;
    std::shared_ptr<IRelocalizer<slam3d::SensorState, std::uint8_t, 32>> relocalizer;

    // local map/graph data
    std::shared_ptr<rgbd::Keyframe> referenceKeyframe;

    slam3d::SensorState currentPose;
    std::shared_ptr<IMap<rgbd::SensorState, rgbd::LandmarkState, rgbd::RgbdOrbKeypointDescriptor>> map;
};
} // namespace mslam

#endif // MSLAM_RGBD_FEATURE_FRONTEND_HPP_
