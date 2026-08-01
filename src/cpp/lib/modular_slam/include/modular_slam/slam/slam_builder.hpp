#ifndef MODULAR_SLAM_SLAM_BUILDER_HPP
#define MODULAR_SLAM_SLAM_BUILDER_HPP

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "modular_slam/slam/sequential_executor.hpp"
#include "modular_slam/slam/slam.hpp"

namespace mslam
{
namespace detail
{
Expected<PipelineGraph> buildPipelineGraph(const std::vector<std::unique_ptr<Module>>& modules,
                                           const std::vector<OutputSpec>& externalInputs,
                                           const std::vector<OutputSpec>& initialStateSlots);
}

class SlamBuilder
{
  public:
    SlamBuilder() = default;

    SlamBuilder(const SlamBuilder&) = delete;
    SlamBuilder& operator=(const SlamBuilder&) = delete;

    SlamBuilder(SlamBuilder&&) noexcept = default;
    SlamBuilder& operator=(SlamBuilder&&) noexcept = default;

    template <typename T>
    SlamBuilder& input(Slot<T> slot)
    {
        mExternalInputs.push_back(produces(slot));
        return *this;
    }

    template <typename T>
    SlamBuilder& setInitialState(Slot<T> slot, T value)
    {
        auto status = mInitialState.set(slot, std::move(value));

        if(!status)
        {
            mBuildError = status.error();
            return *this;
        }

        mInitialStateSlots.push_back(produces(slot));
        return *this;
    }

    SlamBuilder& addModule(std::unique_ptr<Module> module)
    {
        if(!module)
        {
            mBuildError = Error::invalidPipeline("Cannot add null module");
            return *this;
        }

        mModules.push_back(std::move(module));
        return *this;
    }

    SlamBuilder& setExecutor(std::unique_ptr<Executor> executor)
    {
        if(!executor)
        {
            mBuildError = Error::invalidPipeline("Cannot set null executor");
            return *this;
        }

        mExecutor = std::move(executor);
        return *this;
    }

    Expected<Slam> build()
    {
        if(mBuildError.has_value())
        {
            return std::unexpected(*mBuildError);
        }

        if(!mExecutor)
        {
            mExecutor = std::make_unique<mslam::SequentialExecutor>();
        }

        auto graph = detail::buildPipelineGraph(mModules, mExternalInputs, mInitialStateSlots);

        if(!graph)
        {
            return std::unexpected(graph.error());
        }

        return Slam{std::move(mModules), std::move(*graph), std::move(mExecutor), std::move(mInitialState)};
    }

  private:
    std::vector<OutputSpec> mExternalInputs;
    std::vector<OutputSpec> mInitialStateSlots;

    std::vector<std::unique_ptr<Module>> mModules;
    std::unique_ptr<Executor> mExecutor;

    SlotStore mInitialState;

    std::optional<Error> mBuildError;
};

} // namespace mslam
#endif // MODULAR_SLAM_SLAM_BUILDER_HPP
