#ifndef PARAMETERS_HANDLER_HPP_
#define PARAMETERS_HANDLER_HPP_

#include <functional>
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

struct Parameter
{
    ParameterValue currentValue;
    ParameterDefinition definition;
};

using ParametersMap = std::unordered_map<std::string, Parameter>;

class Subscription
{
  public:
    void unsubscribe();
};

class ParameterSubscription : public Subscription
{
  public:
    std::string parameterName();

  private:
};

class ParameterObserver
{
  public:
    virtual void onChange();
    virtual ~ParameterObserver() = default;
};

using ParameterChangeCallback = std::function<void()>;
using NewParameterCallback = std::function<void(const ParameterDefinition&)>;

class ParametersHandlerInterface
{
  public:
    virtual bool registerParameter(const ParameterDefinition& /*paramDefinition*/, const ParameterValue& /*value*/)
    {
        return true;
    }

    virtual bool init() { return true; }
    virtual bool setParameter(const std::string& /*name*/, const ParameterValue& /*newValue*/) = 0;
    virtual std::optional<ParameterValue> getParameter(const std::string& /*name*/) const = 0;
    virtual ParameterSubscription subscribe() = 0;
    virtual Subscription subscribeOnNewParameter(NewParameterCallback observer) = 0;

    virtual ParametersMap allParameters() const = 0;
    virtual ~ParametersHandlerInterface() = default;
};

} // namespace mslam

#endif // PARAMETERS_HANDLER_HPP_
