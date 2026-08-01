#ifndef MODULAR_SLAM_ERROR_HPP
#define MODULAR_SLAM_ERROR_HPP

#include <string>
#include <string_view>

namespace mslam
{

enum class ErrorCode
{
    Ok,
    MissingSlot,
    TypeMismatch,
    SlotConflict,
    OutOfMemory,
    InvalidModuleOutput,
    ModuleFailed,
    ExecutorFailed,
    InvalidPipeline,
    DependencyCycle
};

struct Error
{
    ErrorCode code{ErrorCode::Ok};

    std::string message;
    std::string slotName;
    std::string moduleName;

    static Error missingSlot(std::string_view slot);
    static Error typeMismatch(std::string_view slot, std::string_view module = {});
    static Error slotConflict(std::string_view slot, std::string_view module = {});
    static Error outOfMemory();
    static Error moduleFailed(std::string_view module, Error cause);
    static Error invalidPipeline(std::string message);
    static Error dependencyCycle();
};

} // namespace mslam

#endif // MODULAR_SLAM_ERROR_HPP
