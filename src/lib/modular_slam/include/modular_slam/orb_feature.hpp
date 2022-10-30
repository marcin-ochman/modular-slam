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
    virtual std::vector<KeypointMatch<Eigen::Vector2f>>
    match(FeatureInterface<Eigen::Vector2f>& features) const override;
    virtual std::vector<KeypointMatch<Eigen::Vector2f>> match(boost::span<const Keypoint<Eigen::Vector2f>> keypoints,
                                                              boost::span<const Descriptor> descriptors) const override;
    virtual std::vector<DescriptorMatch> match(boost::span<const Descriptor> descriptors) const override;

    virtual int type() const override { return static_cast<int>(FeatureType::Orb); }
    virtual std::vector<Keypoint<Eigen::Vector2f>> keypoints() const override;
    virtual std::vector<Descriptor> descriptors() const override;
    static std::unique_ptr<OrbFeature> create(cv::Mat descriptors, const std::vector<cv::KeyPoint>& keypoints);

  protected:
    std::vector<KeypointMatch<Eigen::Vector2f>> match(cv::Mat otherDescriptors,
                                                      const std::vector<cv::KeyPoint>& otherKeypoints) const;

    static inline cv::Ptr<cv::DescriptorMatcher> matcher =
        cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

    cv::Mat cvDescriptors;
    std::vector<cv::KeyPoint> cvKeypoints;
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
