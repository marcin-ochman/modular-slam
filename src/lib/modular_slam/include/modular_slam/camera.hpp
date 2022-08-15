#ifndef CAMERA_HPP_
#define CAMERA_HPP_

#include "data_provider.hpp"
#include "modular_slam/basic_types.hpp"
#include "modular_slam/data_provider.hpp"
#include "modular_slam/modular_slam.hpp"
#include <Eigen/src/Core/Matrix.h>

namespace mslam
{

template <typename ImageType>
class Camera : public DataProviderInterface<ImageType>
{
  public:
};

} // namespace mslam

#endif /* CAMERA_HPP_ */
