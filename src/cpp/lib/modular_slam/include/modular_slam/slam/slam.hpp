#ifndef MODULAR_SLAM_SLAM_HPP
#define MODULAR_SLAM_SLAM_HPP

#include <cstdint>
#include <expected>
#include <memory>
#include <utility>
#include <vector>

#include "modular_slam/core/slot_store.hpp"
#include "modular_slam/slam/executor.hpp"
#include "modular_slam/slam/module.hpp"
#include "modular_slam/slam/pipeline_graph.hpp"
#include "modular_slam/slam/step_result.hpp"

namespace mslam
{

class Slam
{
  public:
    Slam(std::vector<std::unique_ptr<Module>> modules, PipelineGraph graph, std::unique_ptr<Executor> executor,
         SlotStore initialState)
        : mModules(std::move(modules)), mGraph(std::move(graph)), mExecutor(std::move(executor)),
          mState(std::move(initialState))
    {
    }

    Slam(const Slam&) = delete;
    Slam& operator=(const Slam&) = delete;

    Slam(Slam&&) noexcept = default;
    Slam& operator=(Slam&&) noexcept = default;

    Expected<StepResult> processStep(SlotStore inputs, TimestampNs timestamp)
    {
        if(!mExecutor)
        {
            return std::unexpected(Error{.code = ErrorCode::ExecutorFailed,
                                         .message = "Slam has no executor",
                                         .slotName = {},
                                         .moduleName = {}});
        }

        StepInfo info{.index = mStepIndex++, .timestamp = timestamp};

        ExecutorStepInput executorInput{.info = info, .stepInputs = std::move(inputs), .state = &mState};

        auto executorResult = mExecutor->runStep(mGraph, mModules, std::move(executorInput));

        if(!executorResult)
        {
            return std::unexpected(executorResult.error());
        }

        auto stateStatus = mState.mergeFrom(std::move(executorResult->stateUpdates));

        if(!stateStatus)
        {
            return std::unexpected(stateStatus.error());
        }

        return StepResult{.info = executorResult->info, .data = std::move(executorResult->stepData)};
    }

    const SlotStore& state() const { return mState; }

    SlotStore& mutableState() { return mState; }

    const PipelineGraph& graph() const { return mGraph; }

    const std::vector<std::unique_ptr<Module>>& modules() const { return mModules; }

    std::uint64_t stepIndex() const { return mStepIndex; }

  private:
    std::vector<std::unique_ptr<Module>> mModules;
    PipelineGraph mGraph;
    std::unique_ptr<Executor> mExecutor;
    SlotStore mState;

    std::uint64_t mStepIndex = 0;
};

} // namespace mslam

#endif // MODULAR_SLAM_SLAM_HPP
