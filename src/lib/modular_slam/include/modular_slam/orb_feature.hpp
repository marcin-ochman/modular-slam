#ifndef MSLAM_ORB_FEATURE_HPP_
#define MSLAM_ORB_FEATURE_HPP_

#include "modular_slam/feature_interface.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/rgb_frame.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <iterator>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>

namespace mslam
{

class OrbFeature : public FeatureInterface<Eigen::Vector2f>
{
  public:
    virtual std::vector<KeypointMatch<Eigen::Vector2f>> match(FeatureInterface<Eigen::Vector2f>& features) override;
    virtual int type() override { return static_cast<int>(FeatureType::Orb); }
    virtual std::vector<Keypoint<Eigen::Vector2f>> getKeypoints() override;

    static std::unique_ptr<OrbFeature> create(cv::Mat descriptors, const std::vector<cv::KeyPoint>& keypoints);

  protected:
    static inline cv::Ptr<cv::DescriptorMatcher> matcher =
        cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;
};

class OrbFeatureDetector : public FeatureDetectorInterface<RgbFrame, Eigen::Vector2f>
{
  public:
    virtual std::unique_ptr<FeatureInterface<Eigen::Vector2f>> detect(const RgbFrame& sensorData) override;

  private:
    static inline cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
};

} // namespace mslam

#endif // MSLAM_ORB_FEATURE_HPP_
