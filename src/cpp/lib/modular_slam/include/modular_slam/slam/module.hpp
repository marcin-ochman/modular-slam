#ifndef MODULAR_SLAM_MODULE_HPP
#define MODULAR_SLAM_MODULE_HPP

#include "modular_slam/slam/module_result.hpp"
#include "modular_slam/slam/module_spec.hpp"
#include "modular_slam/slam/workspace.hpp"

namespace mslam
{

class Module
{
  public:
    Module(const Module&) = default;
    Module(Module&&) = delete;
    Module& operator=(const Module&) = default;
    Module& operator=(Module&&) = delete;

    [[nodiscard]] virtual std::string name() const = 0;
    [[nodiscard]] virtual ModuleSpec spec() const = 0;
    virtual Expected<ModuleResult> run(const Workspace& workspace) = 0;
    virtual ~Module() = default;
};

} // namespace mslam

#endif // MODULAR_SLAM_MODULE_HPP
