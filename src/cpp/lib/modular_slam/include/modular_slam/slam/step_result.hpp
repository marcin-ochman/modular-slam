#ifndef MODULAR_SLAM_STEP_RESULT_HPP
#define MODULAR_SLAM_STEP_RESULT_HPP

#include "modular_slam/core/slot_store.hpp"
#include "modular_slam/slam/step_info.hpp"

namespace mslam
{

struct StepResult
{
    StepInfo info;
    SlotStore data;
};

} // namespace mslam

#endif // MODULAR_SLAM_STEP_RESULT_HPP
