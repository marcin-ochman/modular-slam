#ifndef MODULAR_SLAM_CAMERA_CONCEPTS_HPP_
#define MODULAR_SLAM_CAMERA_CONCEPTS_HPP_

#include "modular_slam/core/vectors.hpp"
#include <concepts>

namespace modular_slam
{
template <typename C>
concept IsCamera = requires { typename C::Scalar; } &&
                   requires(C camera, Vec3<typename C::Scalar> p3, Vec2<typename C::Scalar> p2, typename C::Scalar d) {
                       { camera.project(p3) } -> std::same_as<Vec2<typename C::Scalar>>;
                       { camera.unproject(p2, d) } -> std::same_as<Vec3<typename C::Scalar>>;
                   };
} // namespace modular_slam

#endif // MODULAR_SLAM_CAMERA_CONCEPTS_HPP_
