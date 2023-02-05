#ifndef MSLAM_PNP_HPP_
#define MSLAM_PNP_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/camera_parameters.hpp"
#include "modular_slam/landmark.hpp"

#include <memory>
#include <optional>

namespace mslam
{

template <typename SensorStateType, typename LandmarkStateType>
class IPnpAlgorithm
{
  public:
    virtual std::optional<SensorStateType>
    solvePnp(const std::vector<std::shared_ptr<Landmark<LandmarkStateType>>>& landmarks,
             const std::vector<mslam::Vector2>& imgPoints, const SensorStateType& initial = SensorStateType()) = 0;

    void setCameraParameters(const CameraParameters& newParameters) { cameraParams = newParameters; }
    [[nodiscard]] const CameraParameters& cameraParameters() const { return cameraParams; }

    virtual ~IPnpAlgorithm() = default;

  protected:
    CameraParameters cameraParams;
};
} // namespace mslam

#endif // MSLAM_PNP_HPP_
