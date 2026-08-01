#ifndef MODULAR_SLAM_CORE_VECTORS_HPP_
#define MODULAR_SLAM_CORE_VECTORS_HPP_

namespace mslam
{

template <typename T = float>
struct Vec3
{
    T x, y, z;
};

template <typename T = float>
struct Vec2
{
    T x, y;
};

using Vec3d = Vec3<double>;
using Vec3f = Vec3<float>;
} // namespace mslam

#endif // MODULAR_SLAM_CORE_VECTORS_HPP_
