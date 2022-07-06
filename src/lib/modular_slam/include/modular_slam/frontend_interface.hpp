#ifndef MSLAM_FRONTEND_INTERFACE_HPP_
#define MSLAM_FRONTEND_INTERFACE_HPP_

#include "modular_slam/slam_component.hpp"

namespace mslam
{

template <typename SensorDataType>
class FrontendInterface : public SlamComponent
{
};

} // namespace mslam

#endif // MSLAM_FRONTEND_INTERFACE_HPP_
