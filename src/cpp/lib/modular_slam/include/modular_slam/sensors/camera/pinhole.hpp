#ifndef MODULAR_SLAM_PINHOLE_CAMERA_HPP_
#define MODULAR_SLAM_PINHOLE_CAMERA_HPP_

#include "modular_slam/core/vectors.hpp"
#include "modular_slam/sensors/camera/camera_concepts.hpp"

namespace mslam
{
template <typename T = float>
struct PinholeCamera
{
    using Scalar = T;

    T fx, fy, cx, cy;

    [[nodiscard]] Vec2<T> project(const Vec3<T>& p) const;
    [[nodiscard]] Vec3<T> unproject(const Vec2<T>& uv, double depth) const;
};

template <typename T>
[[nodiscard]] Vec2<T> PinholeCamera<T>::project(const Vec3<T>& p) const
{
    return {p.x * fx / p.z + cx, p.y * fy / p.z + cy};
}

template <typename T>
[[nodiscard]] Vec3<T> PinholeCamera<T>::unproject(const Vec2<T>& uv, double depth) const
{
    return {(uv.x - cx) * depth / fx, (uv.y - cy) * depth / fy, static_cast<T>(depth)};
}

static_assert(IsCamera<PinholeCamera<float>>, "PinholeCamera needs to be compatible with Camera");

} // namespace mslam

#endif // MODULAR_SLAM_PINHOLE_CAMERA_HPP_
