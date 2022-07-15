#ifndef MODULAR_SLAM_SLAM_COMPONENT_HPP_
#define MODULAR_SLAM_SLAM_COMPONENT_HPP_

#include "modular_slam/parameters_handler.hpp"
#include <memory>

namespace mslam
{

class SlamComponent
{
  public:
    SlamComponent(std::shared_ptr<ParametersHandlerInterface> handler = nullptr) : parametersHandler{handler} {}

    void setParameterHandler(std::shared_ptr<ParametersHandlerInterface> newParameterHendler)
    {
        parametersHandler = std::move(newParameterHendler);
    }

    virtual bool init() { return true; }

    virtual ~SlamComponent() {}

  protected:
    std::shared_ptr<ParametersHandlerInterface> parametersHandler;
};

} // namespace mslam

#endif // MODULAR_SLAM_SLAM_COMPONENT_HPP_
