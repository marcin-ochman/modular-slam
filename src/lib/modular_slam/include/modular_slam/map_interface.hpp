#ifndef MSLAM_MAP_INTERFACE_HPP_
#define MSLAM_MAP_INTERFACE_HPP_

#include "modular_slam/slam_component.hpp"

namespace mslam
{

class MapInterface : public SlamComponent
{
  public:
    // virtual bool addPartialMapConstraints();
    // virtual bool globalOptimization();
};

} // namespace mslam

#endif // MAP_HPP_
