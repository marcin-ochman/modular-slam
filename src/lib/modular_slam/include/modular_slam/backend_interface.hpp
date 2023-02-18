#ifndef MSLAM_BACKEND_INTERFACE_H_
#define MSLAM_BACKEND_INTERFACE_H_

#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/keyframe.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/slam3d_types.hpp"
#include "modular_slam/slam_component.hpp"

namespace mslam
{

template <typename LandmarkStateType>
struct MergedLandmark
{
};

template <typename SensorStateType, typename LandmarkStateType>
struct BackendOutput
{
    using LandmarkType = Landmark<LandmarkStateType>;
    using KeyframeType = Keyframe<SensorStateType>;
    using MergedLandmarkType = MergedLandmark<LandmarkStateType>;

    std::vector<LandmarkType> removedLandmarks;
    std::vector<std::shared_ptr<LandmarkType>> updatedLandmarks;
    std::vector<std::shared_ptr<KeyframeType>> updatedKeyframes;
    std::vector<MergedLandmarkType> mergedLandmarks;
};

template <typename SensorStateType, typename LandmarkStateType>
class BackendInterface : public SlamComponent
{
  public:
    using BackendOutputType = BackendOutput<SensorStateType, LandmarkStateType>;

    virtual std::shared_ptr<BackendOutputType>
    process(FrontendOutput<SensorStateType, LandmarkStateType>& frontendOutput) = 0;
};

} // namespace mslam

#endif // BACKEND_INTERFACE_H_
