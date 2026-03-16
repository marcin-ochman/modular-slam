#ifndef MODULAR_SLAM_FISHEYE_HPP_
#define MODULAR_SLAM_FISHEYE_HPP_

namespace modular_slam
{

template <typename T = float>
struct FisheyePolicy
{
    T fx, fy, cx, cy;
};

template <typename T>
[[nodiscard]] Vec2<T> FisheyePolicy<T>::project(const Vec3<T>& p) const
{
    static constexpr T belowZeroThreshold = 1e-6;

    const auto r = std::sqrt(p.x * p.x + p.y * p.y);
    const auto theta = std::atan2(r, p.z);

    if(r < 1e-6)
        return {cx, cy};

    const auto scale = theta / r;
    return {p.x * scale * fx + cx, p.y * scale * fy + cy};
}

template <typename T>
[[nodiscard]] Vec2<T> FisheyePolicy<T>::unproject(const Vec2& uv, double depth) const
{
    const auto mx = (uv.u - cx) / fx;
    const auto my = (uv.v - cy) / fy;
    const auto r = std::sqrt(mx * mx + my * my);

    if(r < 1e-6)
        return {0, 0, depth};

    const auto theta = r;
    const auto sin_theta = std::sin(theta);

    return {depth * sin_theta * (mx / r), depth * sin_theta * (my / r), depth * std::cos(theta)};
}

} // namespace modular_slam

#endif // MODULAR_SLAM_FISHEYE_HPP_
