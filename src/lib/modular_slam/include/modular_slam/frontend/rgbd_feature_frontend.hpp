#ifndef MSLAM_RGBD_FEATURE_FRONTEND_HPP_
#define MSLAM_RGBD_FEATURE_FRONTEND_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/slam3d_types.hpp"
#include "modular_slam/slam_component.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

namespace mslam
{

class RgbdFeatureFrontend : public FrontendInterface<RgbdFrame, slam3d::State, Vector3d>
{
  public:
    using Constraints = typename FrontendInterface<RgbdFrame, slam3d::State, Vector3d>::Constraints;

    RgbdFeatureFrontend();

    std::shared_ptr<Constraints> prepareConstraints(const RgbdFrame& sensorData) override;

  protected:
    bool isNewKeyframeRequired() const;

  private:
    struct ReferenceKeyframeData
    {
        // TODO: move some data to FeatureInterface
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        std::shared_ptr<Keyframe<slam3d::State>> keyframe;
    };

    ReferenceKeyframeData referenceKeyframeData;
    std::shared_ptr<ConstraintsInterface<slam3d::State, Vector3d>> constraints;

    // TODO: move to FeatureInterface
    cv::Ptr<cv::Feature2D> orbDetector;
    cv::Ptr<cv::DescriptorMatcher> matcher;
};
} // namespace mslam

#endif // MSLAM_RGBD_FEATURE_FRONTEND_HPP_
