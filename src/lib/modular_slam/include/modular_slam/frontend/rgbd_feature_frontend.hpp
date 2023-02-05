#ifndef MSLAM_RGBD_FEATURE_FRONTEND_HPP_
#define MSLAM_RGBD_FEATURE_FRONTEND_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/feature_interface.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/keyframe.hpp"
#include "modular_slam/landmark.hpp"
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
class BasicConstraints;

class RgbdFeatureFrontend : public FrontendInterface<RgbdFrame, slam3d::SensorState, Vector3>
{
  public:
    using Constraints = typename FrontendInterface<RgbdFrame, slam3d::SensorState, Vector3>::Constraints;

    RgbdFeatureFrontend(
        std::shared_ptr<IPnpAlgorithm<slam3d::SensorState, Vector3>> tracker,
        std::shared_ptr<IFeatureDetector<RgbFrame, Eigen::Vector2f, float, 32>> detector,
        std::shared_ptr<IFeatureMatcher<Eigen::Vector2f, float, 32>> matcher,
        std::shared_ptr<IFeatureMapComponentsFactory<slam3d::SensorState, slam3d::LandmarkState>> mapComponentsFactory);

    std::shared_ptr<Constraints> prepareConstraints(const RgbdFrame& sensorData) override;
    bool init() override;

    // TODO: remove!!!!
    void visitLandmarks(LandmarkVisitor<Vector3>&) override;
    void visitKeyframes(KeyframeVisitor<slam3d::SensorState>&) override;

  protected:
    [[nodiscard]] std::shared_ptr<rgbd::Keyframe> findBetterReferenceKeyframe(const slam3d::SensorState& currentPose,
                                                                              const RgbdFrame& frame) const;
    [[nodiscard]] bool isBetterReferenceKeyframeNeeded(const std::size_t keyframeLandmarksCount) const;
    [[nodiscard]] bool isNewKeyframeRequired(const std::size_t matchedLandmarks) const;
    [[nodiscard]] bool hasInitialKeyframe() const { return referenceKeyframe != nullptr; }
    [[nodiscard]] bool isLoopClosureNeeded() const;
    void initFirstKeyframe(const RgbdFrame& sensorData,
                           const std::vector<KeypointDescriptor<Eigen::Vector2f, float>>& keypoints);

    std::shared_ptr<rgbd::Keyframe> relocalize();
    std::shared_ptr<Constraints> track(const RgbdFrame& sensorData,
                                       const std::vector<KeypointDescriptor<Eigen::Vector2f, float>>& keypoints);

    std::shared_ptr<rgbd::Keyframe> addKeyframe(const RgbdFrame& sensorData, const slam3d::SensorState& pose);
    [[nodiscard]] std::size_t minMatchedPoints() const;
    [[nodiscard]] std::vector<std::shared_ptr<rgbd::Landmark>>
    findVisibleLocalLandmarks(const slam3d::SensorState& pose, const RgbdFrame& sensorData) const;

    [[nodiscard]] std::vector<KeypointDescriptor<Eigen::Vector2f, float>>
    findKeypointsForNewLandmarks(const std::vector<KeypointDescriptor<Eigen::Vector2f, float>>& keypoints,
                                 const boost::span<std::shared_ptr<rgbd::Landmark>> landmarks) const;

    std::shared_ptr<rgbd::Landmark> addLandmark(std::shared_ptr<rgbd::Keyframe>, const Vector3& state,
                                                const KeypointDescriptor<Eigen::Vector2f, float>& keypoint);

    void bindLandmark(const KeypointDescriptor<Eigen::Vector2f, float>& keypointWithDescriptor,
                      std::shared_ptr<rgbd::Landmark>& landmark);
    void markLandmarkInKeyframe(const std::shared_ptr<rgbd::Landmark>& landmark,
                                const std::shared_ptr<rgbd::Keyframe>& keyframe);

    void addNewLandmarks(const std::vector<KeypointDescriptor<Eigen::Vector2f, float>>& newLandmarkKeypoints,
                         std::shared_ptr<rgbd::Keyframe> newKeyframe, const RgbdFrame& sensorData,
                         std::shared_ptr<ConstraintsInterface<slam3d::SensorState, Vector3>> constraints);

    void updateVisibleLandmarks(const std::vector<std::shared_ptr<rgbd::Landmark>>& landmarksOnFrame,
                                const std::shared_ptr<rgbd::Keyframe> keyframe);

    std::vector<KeypointLandmarkMatch<Eigen::Vector2f, Vector3>>
    matchLandmarks(const std::vector<KeypointDescriptor<Eigen::Vector2f, float>>& keypoints);

    std::pair<std::vector<KeypointDescriptor<Eigen::Vector2f, float>>, std::vector<std::shared_ptr<rgbd::Landmark>>>
    getLandmarksWithKeypoints(std::shared_ptr<rgbd::Keyframe> keyframe);

  private:
    // algorithms
    std::shared_ptr<IFeatureDetector<RgbFrame, Eigen::Vector2f, float, 32>> detector;
    std::shared_ptr<IFeatureMatcher<Eigen::Vector2f, float, 32>> matcher;
    std::shared_ptr<IPnpAlgorithm<slam3d::SensorState, Vector3>> tracker;
    std::shared_ptr<IFeatureMapComponentsFactory<slam3d::SensorState, slam3d::LandmarkState>> mapComponentsFactory;

    // local map/graph data
    std::shared_ptr<rgbd::Keyframe> referenceKeyframe;

    std::vector<std::shared_ptr<rgbd::Keyframe>> keyframes;
    std::vector<std::shared_ptr<rgbd::Landmark>> landmarks;
    std::multimap<std::shared_ptr<rgbd::Landmark>, KeypointDescriptor<Eigen::Vector2f, float>> landmarkDescriptors;
    std::map<std::shared_ptr<rgbd::Keyframe>, std::vector<std::shared_ptr<rgbd::Landmark>>> keyframesWithLandmarks;
    std::map<std::shared_ptr<rgbd::Landmark>, std::vector<std::shared_ptr<rgbd::Keyframe>>> landmarksInKeyframes;
    slam3d::SensorState currentPose;

    std::shared_ptr<BasicConstraints> constraints;
};
} // namespace mslam

#endif // MSLAM_RGBD_FEATURE_FRONTEND_HPP_
