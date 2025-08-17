#ifndef CAMERA_HPP_
#define CAMERA_HPP_

#include "modular_slam/sensors/data_provider.hpp"

namespace mslam
{

template <typename ImageType>
class Camera : public DataProviderInterface<ImageType>
{
  public:
};

} // namespace mslam

#endif /* CAMERA_HPP_ */
