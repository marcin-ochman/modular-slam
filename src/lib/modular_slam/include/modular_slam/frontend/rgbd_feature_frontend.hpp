#ifndef MSLAM_RGBD_FEATURE_FRONTEND_HPP_
#define MSLAM_RGBD_FEATURE_FRONTEND_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/slam3d_types.hpp"
#include "modular_slam/slam_component.hpp"
#include <opencv2/opencv.hpp>

namespace mslam
{

class RgbdFeatureFrontend : public FrontendInterface<RgbdFrame, slam3d::State, Vector3d>
{
  public:
    using Constraints = typename FrontendInterface<RgbdFrame, slam3d::State, Vector3d>::Constraints;

    std::shared_ptr<Constraints> prepareConstraints(const RgbdFrame& sensorData) override;

  protected:
    bool isNewKeyframeRequired() const;

  private:
    struct ReferenceKeyframeData
    {
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        std::shared_ptr<Keyframe<slam3d::State>> keyframe;
    };

    ReferenceKeyframeData referenceKeyframeData;
};
} // namespace mslam

#endif // MSLAM_RGBD_FEATURE_FRONTEND_HPP_
