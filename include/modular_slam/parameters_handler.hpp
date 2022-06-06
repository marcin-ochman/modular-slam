#ifndef PARAMETERS_HANDLER_HPP_
#define PARAMETERS_HANDLER_HPP_

#include <any>
#include <string>

namespace mslam
{

class ParametersHandlerInterface
{
  public:
    virtual bool registerParameter() { return true; }
    virtual bool setParameter(const std::string& name, const std::any value) { return true; }
    virtual std::any getParameter(const std::string& name) const { return std::make_any<std::string>(name); }

    virtual ~ParametersHandlerInterface() {}
};

} // namespace mslam

#endif // PARAMETERS_HANDLER_HPP_
