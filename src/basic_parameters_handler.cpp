#include "modular_slam/basic_parameters_handler.hpp"
#include "modular_slam/parameters_handler.hpp"
#include <optional>

namespace mslam
{

bool BasicParameterHandler::registerParameter(const ParameterDefinition& paramDefinition, const ParameterValue& value)
{
    switch(paramDefinition.type)
    {
        case ParameterType::Number:
            return registerNumberParameter(paramDefinition, std::get<float>(value));
        case ParameterType::Choice:
            return registerChoiceParameter(paramDefinition, std::get<int>(value));
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
