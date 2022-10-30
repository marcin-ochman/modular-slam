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
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/slam3d_types.hpp"
#include "modular_slam/slam_component.hpp"
#include "modular_slam/tracker.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <memory>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

namespace mslam
{

class RgbdFeatureFrontend : public FrontendInterface<RgbdFrame, slam3d::SensorState, Vector3>
{
  public:
    using Constraints = typename FrontendInterface<RgbdFrame, slam3d::SensorState, Vector3>::Constraints;

    RgbdFeatureFrontend(std::shared_ptr<Tracker<slam3d::SensorState, Vector3>> tracker,
                        std::shared_ptr<FeatureDetectorInterface<RgbFrame>> featureDetector);

    std::shared_ptr<Constraints> prepareConstraints(const RgbdFrame& sensorData) override;
    bool init() override;

  protected:
    std::shared_ptr<Keyframe<slam3d::SensorState>> findBestKeyframe() const;
    bool isBetterReferenceKeyframeNeeded(const int keyframeLandmarksCount) const;
    bool isNewKeyframeRequired(const int matchedLandmarks) const;
    bool hasInitialKeyframe() const { return referenceKeyframeData.keyframe != nullptr; }
    bool isLoopClosureNeeded() const;
    void initFirstKeyframe(const RgbdFrame& sensorData, std::unique_ptr<FeatureInterface<Eigen::Vector2f>> features);

    std::shared_ptr<Keyframe<slam3d::SensorState>> relocalize();
    std::shared_ptr<Constraints> track(const RgbdFrame& sensorData, FeatureInterface<Eigen::Vector2f>& features);

    std::shared_ptr<Keyframe<slam3d::SensorState>> addKeyframe(const RgbdFrame& sensorData,
                                                               const slam3d::SensorState& pose,
                                                               FeatureInterface<Eigen::Vector2f>& features);
    std::size_t minMatchedPoints() const;
    std::vector<std::shared_ptr<Landmark<Vector3>>>
    findVisibleLocalLandmarks(const FeatureInterface<Eigen::Vector2f>& features, const slam3d::SensorState& pose,
                              const RgbdFrame& sensorData) const;

    std::vector<Keypoint<Eigen::Vector2i>>
    findKeypointsForNewLandmarks(const FeatureInterface<Eigen::Vector2f>& features,
                                 const boost::span<std::shared_ptr<Landmark<Vector3>>> landmarks) const;

  private:
    struct ReferenceKeyframeData
    {
        std::shared_ptr<FeatureInterface<Eigen::Vector2f>> features;
        std::shared_ptr<Keyframe<slam3d::SensorState>> keyframe;
        RgbdFrame sensorData;
        slam3d::SensorState currentPose;

        std::map<std::shared_ptr<Landmark<Vector3>>, Descriptor> landmarkDescriptors;
    };

    ReferenceKeyframeData referenceKeyframeData;
    std::shared_ptr<ConstraintsInterface<slam3d::SensorState, Vector3>> constraints;

    std::shared_ptr<FeatureDetectorInterface<RgbFrame>> featureDetector;
    std::shared_ptr<Tracker<slam3d::SensorState, Vector3>> tracker;
};
} // namespace mslam

#endif // MSLAM_RGBD_FEATURE_FRONTEND_HPP_
