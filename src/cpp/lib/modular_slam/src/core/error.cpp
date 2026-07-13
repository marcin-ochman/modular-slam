#include "modular_slam/core/error.hpp"

namespace mslam
{

Error Error::missingSlot(std::string_view slot)
{
    return Error{
        .code = ErrorCode::MissingSlot, .message = "Missing slot", .slotName = std::string(slot), .moduleName = {}};
}

Error Error::typeMismatch(std::string_view slot, std::string_view module)
{
    return Error{.code = ErrorCode::TypeMismatch,
                 .message = "Slot type mismatch",
                 .slotName = std::string(slot),
                 .moduleName = std::string(module)};
}

Error Error::slotConflict(std::string_view slot, std::string_view module)
{
    return Error{.code = ErrorCode::SlotConflict,
                 .message = "Slot name reused with incompatible type",
                 .slotName = std::string(slot),
                 .moduleName = std::string(module)};
}

Error Error::dependencyCycle()
{
    return Error{.code = ErrorCode::DependencyCycle,
                 .message = "Dependency cycle detected in pipeline graph",
                 .slotName = {},
                 .moduleName = {}};
}

Error Error::outOfMemory()
{
    return Error{.code = ErrorCode::OutOfMemory, .message = "Out of memory", .slotName = {}, .moduleName = {}};
}

Error Error::moduleFailed(std::string_view module, Error cause)
{
    cause.code = ErrorCode::ModuleFailed;
    cause.moduleName = std::string(module);

    return cause;
}

Error Error::invalidPipeline(std::string message)
{
    return Error{
        .code = ErrorCode::InvalidPipeline, .message = std::move(message), .slotName = {}, .moduleName = {}};
}

} // namespace mslam
