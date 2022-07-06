#ifndef MSLAM_BACKEND_INTERFACE_H_
#define MSLAM_BACKEND_INTERFACE_H_

#include "modular_slam/slam_component.hpp"

namespace mslam
{
template <typename SensorStateType, typename LandmarkStateType>
class BackendInterface : public SlamComponent
{
  public:
};

} // namespace mslam

#endif // BACKEND_INTERFACE_H_
