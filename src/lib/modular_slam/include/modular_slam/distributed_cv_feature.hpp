#ifndef MSLAM_DISTRIBUTED_ORB_FEATURE_HPP_
#define MSLAM_DISTRIBUTED_ORB_FEATURE_HPP_

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

class DistributedOrbOpenCvDetector : public IOrbFeatureDetector
{
  public:
    DistributedOrbOpenCvDetector();
    ~DistributedOrbOpenCvDetector() override;
    virtual std::vector<OrbKeypoint> detect(const RgbFrame& sensorData) override;

  private:
    class OrbExtractorPimpl;

    std::unique_ptr<OrbExtractorPimpl> pimpl;
};

} // namespace mslam

#endif // MSLAM_DISTRIBUTED_ORB_FEATURE_HPP_
