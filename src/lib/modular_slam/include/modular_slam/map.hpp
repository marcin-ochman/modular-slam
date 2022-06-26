#ifndef MAP_HPP_
#define MAP_HPP_

#include "modular_slam/modular_slam.hpp"
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
