#ifndef MSLAM_RGBD_FEATURE_FRONTEND_HPP_
#define MSLAM_RGBD_FEATURE_FRONTEND_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/feature_interface.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/keyframe.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/min_mse_tracker.hpp"
#include "modular_slam/orb_feature.hpp"
#include "modular_slam/rgb_frame.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/slam3d_types.hpp"
#include "modular_slam/slam_component.hpp"
#include "modular_slam/tracker.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cstddef>
#include <memory>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include <modular_slam/rgbd_slam_types.hpp>

namespace mslam
{

class BasicConstraints : public ConstraintsInterface<slam3d::SensorState, Vector3>
{
  public:
    void addConstraint(const LandmarkConstraint<slam3d::SensorState, Vector3> constraint) override;
    void addConstraint(const KeyframeConstraint<slam3d::SensorState, Vector3> constraint) override;
    void visitConstraints(ConstraintVisitor<slam3d::SensorState, Vector3>& visitor) override;

    slam3d::SensorState currentState() override { return state; }
    void setState(const slam3d::SensorState& newState) { state = newState; }

  private:
    std::vector<LandmarkConstraint<slam3d::SensorState, Vector3>> landmarkConstraints;
    std::vector<KeyframeConstraint<slam3d::SensorState, Vector3>> keyframeConstraints;

    slam3d::SensorState state;
};

class RgbdFeatureFrontend : public FrontendInterface<RgbdFrame, slam3d::SensorState, Vector3>
{
  public:
    using Constraints = typename FrontendInterface<RgbdFrame, slam3d::SensorState, Vector3>::Constraints;

    RgbdFeatureFrontend(std::shared_ptr<Tracker<slam3d::SensorState, Vector3>> tracker,
                        std::shared_ptr<IFeatureDetector<RgbFrame, Eigen::Vector2f, float, 32>> detector,
                        std::shared_ptr<IFeatureMatcher<Eigen::Vector2f, float, 32>> matcher);

    std::shared_ptr<Constraints> prepareConstraints(const RgbdFrame& sensorData) override;
    bool init() override;

    // TODO: remove!!!!
    void visitLandmarks(LandmarkVisitor<Vector3>&) override;
    void visitKeyframes(KeyframeVisitor<slam3d::SensorState>&) override;

  protected:
    std::shared_ptr<rgbd::Keyframe> findBetterReferenceKeyframe(const slam3d::SensorState& currentPose,
                                                                const RgbdFrame& frame) const;
    bool isBetterReferenceKeyframeNeeded(const std::size_t keyframeLandmarksCount) const;
    bool isNewKeyframeRequired(const std::size_t matchedLandmarks) const;
    bool hasInitialKeyframe() const { return referenceKeyframeData.keyframe != nullptr; }
    bool isLoopClosureNeeded() const;
    std::shared_ptr<Constraints>
    initFirstKeyframe(const RgbdFrame& sensorData,
                      const std::vector<KeypointDescriptor<Eigen::Vector2f, float>>& keypoints);

    std::shared_ptr<rgbd::Keyframe> relocalize();
    std::shared_ptr<Constraints> track(const RgbdFrame& sensorData,
                                       const std::vector<KeypointDescriptor<Eigen::Vector2f, float>>& keypoints);

    std::shared_ptr<rgbd::Keyframe>
    addKeyframe(const RgbdFrame& sensorData, const slam3d::SensorState& pose,
                const std::vector<KeypointDescriptor<Eigen::Vector2f, float>>& keypoints);
    std::size_t minMatchedPoints() const;
    std::vector<std::shared_ptr<rgbd::Landmark>> findVisibleLocalLandmarks(const slam3d::SensorState& pose,
                                                                           const RgbdFrame& sensorData) const;

    std::vector<KeypointDescriptor<Eigen::Vector2f, float>>
    findKeypointsForNewLandmarks(const std::vector<KeypointDescriptor<Eigen::Vector2f, float>>& keypoints,
                                 const boost::span<std::shared_ptr<rgbd::Landmark>> landmarks) const;

    std::shared_ptr<rgbd::Landmark> addLandmark(std::shared_ptr<rgbd::Keyframe>, const Vector3& state,
                                                const KeypointDescriptor<Eigen::Vector2f, float>& descriptor);

    void bindLandmark(const KeypointDescriptor<Eigen::Vector2f, float> keypointWithDescriptor,
                      std::shared_ptr<rgbd::Landmark> landmark);
    void markLandmarkInKeyframe(std::shared_ptr<rgbd::Landmark> landmark, std::shared_ptr<rgbd::Keyframe> keyframe);

    void addNewLandmarks(const std::vector<KeypointDescriptor<Eigen::Vector2f, float>>& newLandmarkKeypoints,
                         std::shared_ptr<rgbd::Keyframe> newKeyframe, const RgbdFrame& sensorData,
                         std::shared_ptr<ConstraintsInterface<slam3d::SensorState, Vector3>> constraints);

    void updateVisibleLandmarks(const std::vector<std::shared_ptr<rgbd::Landmark>>& landmarksOnFrame,
                                std::shared_ptr<rgbd::Keyframe> keyframe);

    std::vector<KeypointLandmarkMatch<Eigen::Vector2f, Vector3>>
    matchLandmarks(const std::vector<KeypointDescriptor<Eigen::Vector2f, float>>& keypoints);

    std::pair<std::vector<KeypointDescriptor<Eigen::Vector2f, float>>, std::vector<std::shared_ptr<rgbd::Landmark>>>
    getLandmarksWithKeypoints(std::shared_ptr<rgbd::Keyframe> keyframe);

  private:
    struct ReferenceKeyframeData
    {
        std::shared_ptr<rgbd::Keyframe> keyframe;
        slam3d::SensorState currentPose;
    };

    ReferenceKeyframeData referenceKeyframeData;

    // algorithms
    std::shared_ptr<IFeatureDetector<RgbFrame, Eigen::Vector2f, float, 32>> detector;
    std::shared_ptr<IFeatureMatcher<Eigen::Vector2f, float, 32>> matcher;
    std::shared_ptr<Tracker<slam3d::SensorState, Vector3>> tracker;

    // local map/graph data
    std::vector<std::shared_ptr<rgbd::Keyframe>> keyframes;
    std::vector<std::shared_ptr<rgbd::Landmark>> landmarks;
    std::multimap<std::shared_ptr<rgbd::Landmark>, KeypointDescriptor<Eigen::Vector2f, float>> landmarkDescriptors;
    std::map<std::shared_ptr<rgbd::Keyframe>, std::vector<std::shared_ptr<rgbd::Landmark>>> keyframesWithLandmarks;
    std::map<std::shared_ptr<rgbd::Landmark>, std::vector<std::shared_ptr<rgbd::Keyframe>>> landmarksInKeyframes;

    std::vector<float> keypointsDescriptorData;
};
} // namespace mslam

#endif // MSLAM_RGBD_FEATURE_FRONTEND_HPP_
