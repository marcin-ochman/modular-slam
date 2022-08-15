#include "modular_slam/parameters_handler.hpp"

#include <algorithm>

namespace mslam
{

bool isValidChoice(const std::vector<int>& choices, const int value)
{
    return std::find(std::begin(choices), std::end(choices), value) != std::end(choices);
}

ParameterDefinition makeChoiceParameter(const std::string& name, const std::vector<int>& choices)
{
    ParameterDefinition definition{name, ParameterType::Choice, choices, {}};
    return definition;
}

ParameterDefinition makeNumberParameter(const std::string& name, const ParameterRange& range)
{
    ParameterDefinition definition{name, ParameterType::Choice, {}, range};
    return definition;
}

} // namespace mslam
