#ifndef MODULAR_SLAM_BASIC_PARAMETERS_HANDLER_HPP_
#define MODULAR_SLAM_BASIC_PARAMETERS_HANDLER_HPP_

#include "modular_slam/parameters_handler.hpp"

namespace mslam
{

class BasicParameterHandler : public ParametersHandlerInterface
{
    bool registerParameter() override { return true; }
    bool setParameter(const std::string& name, const std::any value) override { return true; }
    std::any getParameter(const std::string& name) const override { return std::make_any<std::string>(name); }

  private:
    std::map<std::string, std::any> parameters;
};

} // namespace mslam

#endif // MODULAR_SLAM_BASIC_PARAMETERS_HANDLER_HPP_
