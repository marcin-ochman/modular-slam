#ifndef PARAMETERS_HANDLER_HPP_
#define PARAMETERS_HANDLER_HPP_

#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace mslam
{

bool isValidChoice(const std::vector<int>& choices, const int value);

enum class ParameterType
{
    Choice,
    Number,
};

struct ParameterRange
{
    float min;
    float max;
    float step;
};

inline bool isInRange(const ParameterRange& range, const float value)
{
    return value >= range.min && value <= range.max;
}

struct ParameterDefinition
{
    std::string name;
    ParameterType type;
    std::vector<int> choices;
    ParameterRange range;
};

ParameterDefinition makeChoiceParameter(const std::string& name, const std::vector<int>& choices);
ParameterDefinition makeNumberParameter(const std::string& name, const ParameterRange& range);

using ParameterValue = std::variant<int, float>;

class ParametersHandlerInterface
{
  public:
    virtual bool registerParameter(const ParameterDefinition& /*paramDefinition*/, const ParameterValue& /*value*/)
    {
        return true;
    }
    virtual bool setParameter(const std::string& /*name*/, const ParameterValue& /*newValue*/) { return true; }

    virtual std::optional<ParameterValue> getParameter(const std::string& /*name*/) const { return 0; }

    virtual ~ParametersHandlerInterface() {}
};

} // namespace mslam

#endif // PARAMETERS_HANDLER_HPP_
