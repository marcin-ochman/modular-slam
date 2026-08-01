#ifndef MODULAR_SLAM_WORKSPACE_HPP_
#define MODULAR_SLAM_WORKSPACE_HPP_

#include "modular_slam/core/slot_store.hpp"

namespace mslam
{

class Workspace
{
  public:
    Workspace(const SlotStore& stepData, const SlotStore* stateData = nullptr)
        : mStepData(stepData), mStateData(stateData)
    {
    }

    template <typename T>
    ExpectedConstRef<T> get(Slot<T> slot) const
    {
        auto stepResult = mStepData.tryGet(slot);

        if(!stepResult)
        {
            return std::unexpected(stepResult.error());
        }

        if(stepResult->has_value())
        {
            return std::cref(*stepResult->value());
        }

        if(mStateData)
        {
            auto stateResult = mStateData->tryGet(slot);

            if(!stateResult)
            {
                return std::unexpected(stateResult.error());
            }

            if(stateResult->has_value())
            {
                return std::cref(*stateResult->value());
            }
        }

        return std::unexpected(Error::missingSlot(slot.name()));
    }

    template <typename T>
    Expected<bool> contains(Slot<T> slot) const
    {
        auto result = get(slot);

        if(!result)
        {
            if(result.error().code == ErrorCode::MissingSlot)
            {
                return false;
            }

            return std::unexpected(result.error());
        }

        return true;
    }

  private:
    const SlotStore& mStepData;
    const SlotStore* mStateData = nullptr;
};

} // namespace mslam

#endif // MODULAR_SLAM_WORKSPACE_HPP_
