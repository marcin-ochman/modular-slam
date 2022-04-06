#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include "modular_slam/types.hpp"
#include "modular_slam/utils.hpp"
#include <optional>

namespace mslam
{
namespace odom
{

template <typename OdometryChildType>
class Odometry
{
};

class OrbBasicOdometry : public Odometry<OrbBasicOdometry>
{

  public:
    using OrbKeypoints = std::vector<cv::KeyPoint>;
    std::optional<Transform> estimatePose(const OrbKeypoints& keypoints);

  private:
    OrbKeypoints previousKeypoints;
};

} // namespace odom
} // namespace mslam

#endif /* ODOMETRY_HPP */
