#ifndef MODULAR_SLAM_BASIC_PARAMETERS_HANDLER_HPP_
#define MODULAR_SLAM_BASIC_PARAMETERS_HANDLER_HPP_

#include "modular_slam/parameters/parameters_handler.hpp"
#include "parameters_handler.hpp"

namespace mslam
{

class BasicParameterHandler : public ParametersHandlerInterface
{
  public:
    bool registerParameter(const ParameterDefinition& paramDefinition, const ParameterValue& value) override;
    bool setParameter(const std::string& name, const ParameterValue& value) override;
    std::optional<ParameterValue> getParameter(const std::string& name) const override;
    ParametersMap allParameters() const override { return parameters; }
    ParameterSubscription subscribe() override;

    Subscription subscribeOnNewParameter(NewParameterCallback observer) override;

  protected:
    bool registerNumberParameter(const ParameterDefinition& paramDefinition, float value);
    bool registerChoiceParameter(const ParameterDefinition& paramDefinition, int value);

    bool setNumberParameter(Parameter& param, float value);
    bool setChoiceParameter(Parameter& param, int value);

  private:
    ParametersMap parameters;

    std::vector<NewParameterCallback> newParameterCallbacks;
};

} // namespace mslam

#endif // MODULAR_SLAM_BASIC_PARAMETERS_HANDLER_HPP_
