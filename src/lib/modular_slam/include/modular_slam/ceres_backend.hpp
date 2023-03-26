#ifndef MSLAM_CERES_BACKEND_HPP_
#define MSLAM_CERES_BACKEND_HPP_

#include "modular_slam/backend_interface.hpp"
#include "modular_slam/basic_types.hpp"
#include "modular_slam/camera_parameters.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/map_interface.hpp"
#include "modular_slam/rgbd_slam_types.hpp"
#include "modular_slam/slam3d_types.hpp"

namespace mslam
{

class CeresVisitor;

class CeresBackend : public BackendInterface<mslam::slam3d::SensorState, mslam::Vector3, rgbd::RgbdKeypoint>
{
  public:
    using FrontendOutputType = FrontendOutput<mslam::slam3d::SensorState, mslam::Vector3, rgbd::RgbdKeypoint>;
    using MapType = IMap<mslam::slam3d::SensorState, mslam::Vector3, mslam::rgbd::RgbdKeypoint>;

    BackendOutputType process(FrontendOutputType& frontendOutput) override;
    void setCameraParameters(const CameraParameters& newCameraParameters) { cameraParameters = newCameraParameters; }
    void setMap(std::shared_ptr<MapType> newMap) { map = std::move(newMap); }

    bool init() override;

    [[nodiscard]] int lbaMaxIterations() const;

  private:
    [[nodiscard]] bool needsGlobalBundleAdjustment(const FrontendOutputType& frontendOutput) const;
    [[nodiscard]] bool needsLocalBundleAdjustment(const FrontendOutputType& frontendOutput) const;
    std::vector<rgbd::KeyframeLandmarkObservation> getObservationsForLBA() const;
    BackendOutputType localBundleAdjustment(const FrontendOutputType& frontendOutput);
    BackendOutputType globalBundleAdjustment(const FrontendOutputType& frontendOutput);
    BackendOutputType bundleAdjustment(const MapVisitingParams& visitingParams);

    BackendOutputType createOutput(const CeresVisitor& visitor) const;

    CameraParameters cameraParameters; // = {{319.5f, 239.5f}, {525, 525}, 1.f / 5000.f};

    std::shared_ptr<MapType> map;
};
} // namespace mslam

#endif // MSLAM_CERES_BACKEND_HPP_
