#include "modular_slam/parameters/basic_parameters_handler.hpp"
#include "include/modular_slam/parameters/basic_parameters_handler.hpp"
#include "include/modular_slam/parameters/parameters_handler.hpp"
#include <algorithm>
#include <optional>

namespace mslam
{

bool BasicParameterHandler::registerParameter(const ParameterDefinition& paramDefinition, const ParameterValue& value)
{
    bool result = false;

    switch(paramDefinition.type)
    {
        case ParameterType::Number:
            result = registerNumberParameter(paramDefinition, std::get<float>(value));
            break;
        case ParameterType::Choice:
            result = registerChoiceParameter(paramDefinition, std::get<int>(value));
            break;
    }

    if(result)
    {
        std::for_each(std::begin(newParameterCallbacks), std::end(newParameterCallbacks),
                      [&paramDefinition](auto& f) { f(paramDefinition); });
    }

    return false;
}

bool BasicParameterHandler::setParameter(const std::string& name, const ParameterValue& value)
{
    auto it = parameters.find(name);
    auto found = it != std::end(parameters);

    if(found)
    {
        switch(it->second.definition.type)
        {
            case ParameterType::Number:
                return setNumberParameter(it->second, std::get<float>(value));
            case ParameterType::Choice:
                return setChoiceParameter(it->second, std::get<int>(value));
        }
    }

    return found;
}

bool BasicParameterHandler::setNumberParameter(Parameter& param, const float value)
{
    if(isInRange(param.definition.range, value))
        return false;

    param.currentValue = value;

    return false;
}

bool BasicParameterHandler::setChoiceParameter(Parameter& param, const int value)
{

    if(isValidChoice(param.definition.choices, value))
    {
        param.currentValue = value;
        return true;
    }

    return false;
}

std::optional<ParameterValue> BasicParameterHandler::getParameter(const std::string& name) const
{
    auto it = parameters.find(name);
    auto found = it != std::end(parameters);

    return found ? std::make_optional(it->second.currentValue) : std::nullopt;
}

ParameterSubscription BasicParameterHandler::subscribe()
{
    return ParameterSubscription();
}

Subscription BasicParameterHandler::subscribeOnNewParameter(NewParameterCallback observer)
{
    // TODO: use boost::signal
    newParameterCallbacks.emplace_back(std::move(observer));

    return Subscription();
}

bool BasicParameterHandler::registerNumberParameter(const ParameterDefinition& paramDefinition, float value)
{
    auto it = parameters.find(paramDefinition.name);
    auto result = false;
    auto found = it != std::end(parameters);

    if(!found && isInRange(paramDefinition.range, value))
    {
        Parameter param = {value, paramDefinition};
        parameters[paramDefinition.name] = param;
        result = true;
    }

    return result;
}

bool BasicParameterHandler::registerChoiceParameter(const ParameterDefinition& paramDefinition, int value)
{
    auto it = parameters.find(paramDefinition.name);
    auto result = false;
    auto found = it != std::end(parameters);

    if(!found && isValidChoice(paramDefinition.choices, value))
    {
        Parameter param = {value, paramDefinition};
        parameters[paramDefinition.name] = param;
        result = true;
    }

    return result;
}

} // namespace mslam
