#ifndef MODULAR_SLAM_MODULE_SPEC_HPP
#define MODULAR_SLAM_MODULE_SPEC_HPP

#include "modular_slam/core/slot.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace mslam
{

enum class InputPolicy
{
    Required,      // missing input is an error
    Optional,      // module can run without it
    SkipIfMissing, // if missing, module is skipped for this step
};

enum class MergePolicy
{
    SingleWriter, // only one module may write this slot
    Append,       // multiple modules append to collection
    Reduce,       // reducer combines outputs
    Latest,       // latest wins, mostly diagnostics/viewer
};

struct InputSpec
{
    SlotId id;
    std::string_view name;
    TypeToken type;
    InputPolicy policy = InputPolicy::Required;
};

struct OutputSpec
{
    SlotId id;
    std::string_view name;
    TypeToken type;
    MergePolicy mergePolicy = MergePolicy::SingleWriter;
};

template <typename T>
InputSpec required(Slot<T> slot)
{
    return InputSpec{.id = slot.id(), .name = slot.name(), .type = slot.type(), .policy = InputPolicy::Required};
}

template <typename T>
InputSpec optional(Slot<T> slot)
{
    return InputSpec{.id = slot.id(), .name = slot.name(), .type = slot.type(), .policy = InputPolicy::Optional};
}

template <typename T>
InputSpec skipIfMissing(Slot<T> slot)
{
    return InputSpec{.id = slot.id(), .name = slot.name(), .type = slot.type(), .policy = InputPolicy::SkipIfMissing};
}

template <typename T>
OutputSpec produces(Slot<T> slot, MergePolicy policy = MergePolicy::SingleWriter)
{
    return OutputSpec{.id = slot.id(), .name = slot.name(), .type = slot.type(), .mergePolicy = policy};
}

struct ExecutionHints
{
    bool parallelizable = true;
    bool stateful = false;
    int priority = 0;
};

struct ModuleSpec
{
    std::string name;

    std::vector<InputSpec> inputs;
    std::vector<OutputSpec> outputs;

    ExecutionHints execution;
};

} // namespace mslam

#endif // MODULAR_SLAM_MODULE_SPEC_HPP
