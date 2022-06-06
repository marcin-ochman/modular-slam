#include "modular_slam/basic_parameters_handler.hpp"

namespace mslam
{
bool BasicParameterHandler::registerParameter() override
{
    return true;
}
bool BasicParameterHandler::setParameter(const std::string& name, const std::any value) override
{
    return true;
}
std::any BasicParameterHandler::getParameter(const std::string& name) const override
{
    return std::make_any<std::string>(name);
}

} // namespace mslam
