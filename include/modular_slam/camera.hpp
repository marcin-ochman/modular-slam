#ifndef CAMERA_HPP_
#define CAMERA_HPP_

#include "data_provider.hpp"
#include "modular_slam/data_provider.hpp"
#include "modular_slam/modular_slam.hpp"


namespace mslam
{

template <typename ImageType>
class Camera : public DataProvider<ImageType>
{
  public:

};

} // namespace mslam

#endif /* CAMERA_HPP_ */
