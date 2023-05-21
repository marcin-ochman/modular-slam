#ifndef MSLAM_ORB_FEATURE_HPP_
#define MSLAM_ORB_FEATURE_HPP_

#include "modular_slam/feature_interface.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/rgb_frame.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <iterator>

namespace mslam
{

using IOrbFeatureDetector = IFeatureDetector<RgbFrame, float, 32>;
using IOrbMatcher = IFeatureMatcher<float, 32>;
using OrbKeypoint = KeypointDescriptor<float, 32>;

class OrbOpenCvDetector : public IOrbFeatureDetector
{
  public:
    OrbOpenCvDetector();
    ~OrbOpenCvDetector() override;
    virtual std::vector<OrbKeypoint> detect(const RgbFrame& sensorData) override;

  private:
    class Pimpl;

    std::unique_ptr<Pimpl> pimpl;
};

class OrbOpenCvMatcher : public IOrbMatcher
{
  public:
    OrbOpenCvMatcher();
    std::vector<DescriptorMatch> match(const std::vector<OrbKeypoint>& firstDescriptors,
                                       const std::vector<OrbKeypoint>& secondDescriptors) override;

    ~OrbOpenCvMatcher() override;

  private:
    class Pimpl;

    std::unique_ptr<Pimpl> pimpl;
};

} // namespace mslam

#endif // MSLAM_ORB_FEATURE_HPP_
