#ifndef MSLAM_LANDMARK_HPP_
#define MSLAM_LANDMARK_HPP_

#include "modular_slam/basic_types.hpp"

namespace mslam
{

using LandmarkId = Id;

template <typename StateType>
struct Landmark
{
    Id id;
    StateType state;
};

} // namespace mslam

#endif // MSLAM_LANDMARK_HPP_
