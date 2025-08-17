#ifndef MSLAM_BACKEND_INTERFACE_H_
#define MSLAM_BACKEND_INTERFACE_H_

#include "modular_slam/backend/backend_output.hpp"
#include "modular_slam/slam_component.hpp"
#include "modular_slam/types/frontend_output.hpp"
#include "modular_slam/types/slam3d_types.hpp"
#include <unordered_set>

namespace mslam
{

template <typename SensorStateType, typename LandmarkStateType, typename ObservationType>
class BackendInterface : public SlamComponent
{
  public:
    using BackendOutputType = BackendOutput<SensorStateType, LandmarkStateType, ObservationType>;

    virtual BackendOutputType
    process(FrontendOutput<SensorStateType, LandmarkStateType, ObservationType>& frontendOutput) = 0;
};

} // namespace mslam

#endif // BACKEND_INTERFACE_H_
