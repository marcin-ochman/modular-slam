#ifndef MODULAR_SLAM_SEQUENTIAL_EXECUTOR_HPP
#define MODULAR_SLAM_SEQUENTIAL_EXECUTOR_HPP

#include "modular_slam/core/expected.hpp"
#include "modular_slam/core/slot_store.hpp"
#include "modular_slam/slam/executor.hpp"
#include "modular_slam/slam/pipeline_graph.hpp"

namespace mslam
{

class SequentialExecutor final : public Executor
{
  public:
    Expected<ExecutorStepResult> runStep(const PipelineGraph& graph,
                                         const std::vector<std::unique_ptr<Module>>& modules,
                                         ExecutorStepInput input) override
    {
        SlotStore stepData = std::move(input.stepInputs);
        SlotStore stateUpdates;

        for(const auto& level : graph.levels)
        {
            for(std::size_t graphNodeIndex : level)
            {
                const ModuleNode& node = graph.nodes[graphNodeIndex];
                Module& module = *modules[node.moduleIndex];

                auto decision = shouldRun(node.spec, stepData, input.state);
                if(!decision)
                {
                    return std::unexpected(Error::moduleFailed(node.spec.name, decision.error()));
                }

                if(*decision == RuntimeDecision::Skip)
                {
                    continue;
                }

                Workspace workspace(stepData, input.state);

                auto moduleResult = module.run(workspace);
                if(!moduleResult)
                {
                    return std::unexpected(Error::moduleFailed(node.spec.name, moduleResult.error()));
                }

                ModuleResult result = std::move(*moduleResult);

                auto mergeStepStatus = stepData.mergeFrom(std::move(result).takeOutputs());

                if(!mergeStepStatus)
                {
                    return std::unexpected(Error::moduleFailed(node.spec.name, mergeStepStatus.error()));
                }

                auto mergeStateStatus = stateUpdates.mergeFrom(std::move(result).takeOutputs());

                if(!mergeStateStatus)
                {
                    return std::unexpected(Error::moduleFailed(node.spec.name, mergeStateStatus.error()));
                }
            }
        }

        return ExecutorStepResult{
            .info = input.info, .stepData = std::move(stepData), .stateUpdates = std::move(stateUpdates)};
    }

  private:
    enum class RuntimeDecision
    {
        Run,
        Skip,
    };

    Expected<RuntimeDecision> shouldRun(const ModuleSpec& spec, const SlotStore& stepData, const SlotStore* state) const
    {
        Workspace workspace(stepData, state);

        for(const auto& input : spec.inputs)
        {
            // Workspace does not have a dynamic Slot<T> here because we only have
            // InputSpec. In a production implementation I would add:
            //
            //   SlotStore::containsIdAndType(SlotId, TypeToken)
            //
            // For now, assume the graph compiler has already validated static
            // dependencies. Runtime missing checks matter mostly for external
            // inputs and SkipIfMissing sensors.

            const bool existsInStep = stepData.containsId(input.id);
            const bool existsInState = state && state->containsId(input.id);
            const bool exists = existsInStep || existsInState;

            if(!exists && input.policy == InputPolicy::Required)
            {
                // TODO
                // return std::unexpected(Error::missingSlot(input.name, spec.name));
            }

            if(!exists && input.policy == InputPolicy::SkipIfMissing)
            {
                return RuntimeDecision::Skip;
            }
        }

        return RuntimeDecision::Run;
    }
};

} // namespace mslam
#endif // MODULAR_SLAM_SEQUENTIAL_EXECUTOR_HPP
