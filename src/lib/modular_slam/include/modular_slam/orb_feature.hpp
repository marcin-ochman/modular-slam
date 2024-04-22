#ifndef MSLAM_ORB_FEATURE_HPP_
#define MSLAM_ORB_FEATURE_HPP_

#include "modular_slam/frontend/feature/feature_interface.hpp"
#include "modular_slam/types/rgb_frame.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>

namespace mslam
{

using IOrbFeatureDetector = IFeatureDetector<RgbFrame, std::uint8_t, 32>;
using IOrbMatcher = IFeatureMatcher<std::uint8_t, 32>;
using OrbKeypoint = KeypointDescriptor<std::uint8_t, 32>;

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
