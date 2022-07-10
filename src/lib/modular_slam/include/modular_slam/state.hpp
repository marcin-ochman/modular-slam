#ifndef MSLAM_STATE_HPP_
#define MSLAM_STATE_HPP_

namespace mslam
{

template <typename PositionType, typename OrientationType>
struct State
{
    PositionType position;
    OrientationType orientation;
};

} // namespace mslam

#endif // MSLAM_STATE_HPP_
