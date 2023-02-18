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

using IOrbFeatureDetector = IFeatureDetector<RgbFrame, float, 32>;
using IOrbMatcher = IFeatureMatcher<float, 32>;
using OrbKeypoint = KeypointDescriptor<float, 32>;

class OrbOpenCvDetector : public IOrbFeatureDetector
{
  public:
    virtual std::vector<OrbKeypoint> detect(const RgbFrame& sensorData) override;

  private:
    static inline cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
};

class OrbOpenCvMatcher : public IOrbMatcher
{
  public:
    std::vector<DescriptorMatch> match(const std::vector<OrbKeypoint>& firstDescriptors,
                                       const std::vector<OrbKeypoint>& secondDescriptors) override;

  private:
    static inline cv::Ptr<cv::DescriptorMatcher> matcher =
        cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
};

} // namespace mslam

#endif // MSLAM_ORB_FEATURE_HPP_
