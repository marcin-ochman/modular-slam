#ifndef MODULAR_SLAM_MODULE_RESULT_HPP
#define MODULAR_SLAM_MODULE_RESULT_HPP

#include "modular_slam/core/slot_store.hpp"

namespace mslam
{

class ModuleResult
{
  public:
    template <typename T>
    Status set(Slot<T> slot, T value)
    {
        return mOutputs.set(slot, std::move(value));
    }

    SlotStore takeOutputs() && { return std::move(mOutputs); }

  private:
    SlotStore mOutputs;
};

} // namespace mslam
#endif // MODULAR_SLAM_MODULE_RESULT_HPP
