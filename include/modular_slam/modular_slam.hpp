#ifndef MODULAR_SLAM_HPP_
#define MODULAR_SLAM_HPP_

#include "modular_slam/loop_detection.hpp"
#include "modular_slam/parameters_handler.hpp"
#include "modular_slam/types.hpp"

#include <any>
#include <cstdint>
#include <memory>
#include <vector>

namespace mslam
{

class SlamComponent
{
  public:
    SlamComponent(std::shared_ptr<ParametersHandlerInterface> handler) : parametersHandler{handler} {}

  private:
    std::shared_ptr<ParametersHandlerInterface> parametersHandler;
};

} // namespace mslam

#endif /* MODULAR_SLAM_HPP_ */
