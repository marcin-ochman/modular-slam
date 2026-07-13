#ifndef MODULAR_SLAM_FISHEYE_HPP_
#define MODULAR_SLAM_FISHEYE_HPP_

#include "modular_slam/core/vectors.hpp"

#include <cmath>

namespace mslam
{

template <typename T = float>
struct FisheyePolicy
{
    T fx, fy, cx, cy;

    [[nodiscard]] Vec2<T> project(const Vec3<T>& p) const;
    [[nodiscard]] Vec3<T> unproject(const Vec2<T>& uv, double depth) const;
};

template <typename T>
[[nodiscard]] Vec2<T> FisheyePolicy<T>::project(const Vec3<T>& p) const
{
    static constexpr T belowZeroThreshold = static_cast<T>(1e-6);

    const auto r = std::sqrt(p.x * p.x + p.y * p.y);
    const auto theta = std::atan2(r, p.z);

    if(r < belowZeroThreshold)
    {
        return {cx, cy};
    }

    const auto scale = theta / r;
    return {p.x * scale * fx + cx, p.y * scale * fy + cy};
}

template <typename T>
[[nodiscard]] Vec3<T> FisheyePolicy<T>::unproject(const Vec2<T>& uv, double depth) const
{
    const auto mx = (uv.x - cx) / fx;
    const auto my = (uv.y - cy) / fy;
    const auto r = std::sqrt(mx * mx + my * my);

    if(r < static_cast<T>(1e-6))
    {
        return {0, 0, static_cast<T>(depth)};
    }

    const auto theta = r;
    const auto sin_theta = std::sin(theta);

    return {depth * sin_theta * (mx / r), depth * sin_theta * (my / r), depth * std::cos(theta)};
}

} // namespace mslam

#endif // MODULAR_SLAM_FISHEYE_HPP_
