#ifndef MSLAM_RGBD_FEATURE_FRONTEND_HPP_
#define MSLAM_RGBD_FEATURE_FRONTEND_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/feature_interface.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/keyframe.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/loop_detection.hpp"
#include "modular_slam/map_interface.hpp"
#include "modular_slam/orb_feature.hpp"
#include "modular_slam/pnp.hpp"
#include "modular_slam/rgb_frame.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/slam3d_types.hpp"
#include "modular_slam/slam_component.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <cstddef>
#include <memory>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include <modular_slam/rgbd_slam_types.hpp>

namespace mslam
{

class RgbdFeatureFrontend : public FrontendInterface<RgbdFrame, slam3d::SensorState, Vector3>
{
  public:
    using FrontendOutputType = typename FrontendInterface<RgbdFrame, slam3d::SensorState, Vector3>::FrontendOutputType;

    RgbdFeatureFrontend(
        std::shared_ptr<IPnpAlgorithm<slam3d::SensorState, Vector3>> tracker,
        std::shared_ptr<IFeatureDetector<RgbFrame, float, 32>> detector,
        std::shared_ptr<IFeatureMatcher<float, 32>> matcher,
        std::shared_ptr<IMapComponentsFactory<slam3d::SensorState, slam3d::LandmarkState>> mapComponentsFactory);

    std::shared_ptr<FrontendOutputType> processSensorData(const RgbdFrame& sensorData) override;
    bool init() override;

  protected:
    [[nodiscard]] std::shared_ptr<rgbd::Keyframe> findBetterReferenceKeyframe(const slam3d::SensorState& currentPose,
                                                                              const RgbdFrame& frame) const;
    [[nodiscard]] bool isBetterReferenceKeyframeNeeded(const std::size_t keyframeLandmarksCount) const;
    [[nodiscard]] bool isNewKeyframeRequired(const std::size_t matchedLandmarks) const;
    [[nodiscard]] bool hasInitialKeyframe() const { return referenceKeyframe != nullptr; }
    [[nodiscard]] bool isLoopClosureNeeded() const;
    void initFirstKeyframe(const RgbdFrame& sensorData, const std::vector<KeypointDescriptor<float, 32>>& keypoints);

    std::shared_ptr<rgbd::Keyframe> relocalize();
    bool track(const RgbdFrame& sensorData, const std::vector<KeypointDescriptor<float, 32>>& keypoints);

    std::shared_ptr<rgbd::Keyframe> addKeyframe(const RgbdFrame& sensorData, const slam3d::SensorState& pose);
    [[nodiscard]] std::size_t minMatchedPoints() const;
    [[nodiscard]] std::vector<std::shared_ptr<rgbd::Landmark>>
    findVisibleLocalLandmarks(const slam3d::SensorState& pose, const RgbdFrame& sensorData) const;

    [[nodiscard]] std::vector<KeypointDescriptor<float, 32>>
    findKeypointsForNewLandmarks(const std::vector<KeypointDescriptor<float, 32>>& keypoints,
                                 const boost::span<std::shared_ptr<rgbd::Landmark>> landmarks) const;

    std::shared_ptr<rgbd::Landmark> addLandmark(std::shared_ptr<rgbd::Keyframe>&, const Vector3& state,
                                                const KeypointDescriptor<float, 32>& keypoint);

    void bindLandmark(const KeypointDescriptor<float, 32>& keypointWithDescriptor,
                      std::shared_ptr<rgbd::Landmark>& landmark);
    void markLandmarkInKeyframe(const std::shared_ptr<rgbd::Landmark>& landmark,
                                const std::shared_ptr<rgbd::Keyframe>& keyframe);

    void addNewLandmarks(const std::vector<KeypointDescriptor<float, 32>>& newLandmarkKeypoints,
                         std::shared_ptr<rgbd::Keyframe> newKeyframe, const RgbdFrame& sensorData,
                         std::shared_ptr<FrontendOutputType> constraints);

    void updateVisibleLandmarks(const std::vector<std::shared_ptr<rgbd::Landmark>>& landmarksOnFrame,
                                const std::shared_ptr<rgbd::Keyframe>& keyframe);

    std::vector<KeypointLandmarkMatch<Vector3>>
    matchLandmarks(const std::vector<KeypointDescriptor<float, 32>>& keypoints);

    std::pair<std::vector<KeypointDescriptor<float, 32>>, std::vector<std::shared_ptr<rgbd::Landmark>>>
    getLandmarksWithKeypoints(std::shared_ptr<rgbd::Keyframe> keyframe);
    void closeLoop(std::shared_ptr<Keyframe<slam3d::SensorState>>& loopCandidate);

  private:
    // algorithms
    std::shared_ptr<IFeatureDetector<RgbFrame, float, 32>> detector;
    std::shared_ptr<IFeatureMatcher<float, 32>> matcher;
    std::shared_ptr<IPnpAlgorithm<slam3d::SensorState, Vector3>> tracker;
    std::shared_ptr<IMapComponentsFactory<slam3d::SensorState, slam3d::LandmarkState>> mapComponentsFactory;
    std::shared_ptr<ILoopDetector<slam3d::SensorState>> loopDetector;

    // local map/graph data
    std::shared_ptr<rgbd::Keyframe> referenceKeyframe;

    std::vector<std::shared_ptr<rgbd::Keyframe>> keyframes;
    std::vector<std::shared_ptr<rgbd::Landmark>> landmarks;
    std::multimap<std::shared_ptr<rgbd::Landmark>, KeypointDescriptor<float>> landmarkDescriptors;
    std::map<std::shared_ptr<rgbd::Keyframe>, std::vector<std::shared_ptr<rgbd::Landmark>>> keyframesWithLandmarks;
    std::map<std::shared_ptr<rgbd::Landmark>, std::vector<std::shared_ptr<rgbd::Keyframe>>> landmarksInKeyframes;
    slam3d::SensorState currentPose;

    std::shared_ptr<FrontendOutputType> output;
};
} // namespace mslam

#endif // MSLAM_RGBD_FEATURE_FRONTEND_HPP_
