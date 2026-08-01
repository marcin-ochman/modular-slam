#ifndef MODULAR_SLAM_EXECUTOR_HPP
#define MODULAR_SLAM_EXECUTOR_HPP

#include "modular_slam/slam/module.hpp"
#include "modular_slam/slam/pipeline_graph.hpp"
#include "modular_slam/slam/step_info.hpp"

namespace mslam
{

struct ExecutorStepInput
{
    StepInfo info;
    SlotStore stepInputs;
    const SlotStore* state = nullptr;
};

struct ExecutorStepResult
{
    StepInfo info;
    SlotStore stepData;
    SlotStore stateUpdates;
};

class Executor
{
  public:
    virtual ~Executor() = default;

    virtual Expected<ExecutorStepResult> runStep(const PipelineGraph& graph,
                                                 const std::vector<std::unique_ptr<Module>>& modules,
                                                 ExecutorStepInput input) = 0;
};
} // namespace mslam
#endif //  MODULAR_SLAM_EXECUTOR_HPP
