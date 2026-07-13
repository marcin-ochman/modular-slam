#ifndef MODULAR_SLAM_VISUAL_FRONTEND_HPP_
#define MODULAR_SLAM_VISUAL_FRONTEND_HPP_

#include <concepts>
#include <optional>

#include "modular_slam/core/concept_utils.hpp"
#include "modular_slam/core/constraint.hpp"
#include "modular_slam/core/pose.hpp"
#include "modular_slam/sensors/camera_frame.hpp"

namespace mslam::frontend
{

template <typename F>
concept IsVisualFrontend = requires(F frontend, typename F::InputData data, Pose initialPose) {
    { frontend.process(data, initialPose) } -> std::same_as<std::optional<Constraint>>;
} && IsDerivedFromTemplate<typename F::InputData, CameraFrame>;

} // namespace mslam::frontend

#endif // MODULAR_SLAM_VISUAL_FRONTEND_HPP_
