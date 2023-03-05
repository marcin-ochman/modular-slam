#ifndef MODULAR_SLAM_CV_PNP_RANSAC_HPP_
#define MODULAR_SLAM_CV_PNP_RANSAC_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/pnp.hpp"
#include "modular_slam/slam3d_types.hpp"

namespace mslam
{

class OpenCvRansacPnp : public IPnpAlgorithm<slam3d::SensorState, Vector3>
{
  public:
    std::optional<PnpResult> solvePnp(const std::vector<std::shared_ptr<Landmark<Vector3>>>& landmarks,
                                      const std::vector<Vector2>& sensorPoints,
                                      const slam3d::SensorState& initial = slam3d::SensorState()) override;
};

} // namespace mslam

#endif // MODULAR_SLAM_CV_PNP_RANSAC_HPP_
