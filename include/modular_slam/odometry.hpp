#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include "modular_slam/types.hpp"

namespace mslam
{
namespace odom
{

class Odom
{
  public:
    virtual void registerNextFrame(const Frame& newFrame) = 0;
    virtual Transform3<float> calculateTransform() = 0;
};

} // namespace odom
} // namespace mslam

#endif /* ODOMETRY_HPP */
