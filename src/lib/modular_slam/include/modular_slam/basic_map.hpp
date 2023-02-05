#ifndef BASIC_MAP_HPP_
#define BASIC_MAP_HPP_

#include "modular_slam/basic_types.hpp"
#include "modular_slam/map_interface.hpp"
#include "modular_slam/slam3d_types.hpp"

namespace mslam
{
class BasicMap : public MapInterface<slam3d::SensorState, slam3d::LandmarkState>
{
  public:
    void update(const std::shared_ptr<Constraints> constraints) override;
};
} // namespace mslam

#endif // BASIC_MAP_HPP_
