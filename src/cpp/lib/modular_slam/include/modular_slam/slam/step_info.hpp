#ifndef MODULAR_SLAM_STEP_INFO_HPP
#define MODULAR_SLAM_STEP_INFO_HPP

#include <cstdint>

namespace mslam
{

using TimestampNs = std::int64_t;

struct StepInfo
{
    std::uint64_t index = 0;
    TimestampNs timestamp = 0;
};

} // namespace mslam

#endif // MODULAR_SLAM_STEP_INFO_HPP
