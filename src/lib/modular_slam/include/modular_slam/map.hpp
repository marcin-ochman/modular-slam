#ifndef MAP_HPP_
#define MAP_HPP_

#include "modular_slam/slam_component.hpp"

namespace mslam
{

/*!
 *
 *
 */
class Map : public SlamComponent
{
  public:
    virtual bool globalOptimization();
};

} // namespace mslam

#endif // MAP_HPP_
