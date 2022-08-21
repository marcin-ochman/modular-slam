#ifndef MSLAM_CERES_BACKEND_HPP_
#define MSLAM_CERES_BACKEND_HPP_

#include "modular_slam/backend_interface.hpp"
#include "modular_slam/basic_types.hpp"
#include "modular_slam/slam3d_types.hpp"
namespace mslam
{
class CeresBackend : public BackendInterface<mslam::slam3d::State, mslam::Vector3d>
{
  public:
    void optimize(ConstraintsInterface<mslam::slam3d::State, mslam::Vector3d>& constraints) override;
};
} // namespace mslam

#endif // MSLAM_CERES_BACKEND_HPP_