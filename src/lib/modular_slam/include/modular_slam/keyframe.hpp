#ifndef MSLAM_KEYFRAME_HPP_
#define MSLAM_KEYFRAME_HPP_

#include "modular_slam/basic_types.hpp"

namespace mslam
{
template <typename StateType>
struct Keyframe
{
    Id id;
    StateType state;
};

} // namespace mslam

#endif // MSLAM_KEYFRAME_HPP_
