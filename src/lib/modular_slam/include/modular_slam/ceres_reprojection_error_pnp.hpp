#ifndef MSLAM_CERES_REPROJECTION_ERROR_PNP_HPP_
#define MSLAM_CERES_REPROJECTION_ERROR_PNP_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/pnp.hpp"
#include "modular_slam/slam3d_types.hpp"

namespace mslam
{

class MinMseTracker : public IPnpAlgorithm<slam3d::SensorState, Vector3>
{
  public:
    std::optional<PnpResult> solvePnp(const std::vector<std::shared_ptr<Landmark<Vector3>>>& landmarks,
                                      const std::vector<Vector2>& sensorPoints,
                                      const slam3d::SensorState& initial = slam3d::SensorState()) override;
};

} // namespace mslam

#endif // MSLAM_CERES_REPROJECTION_ERROR_PNP_HPP_
