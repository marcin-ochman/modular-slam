#ifndef MODULAR_SLAM_VISUAL_FRONTEND_HPP_
#define MODULAR_SLAM_VISUAL_FRONTEND_HPP_

#include <concepts>

#include "modular_slam/core/concept_utils.hpp"
#include "modular_slam/core/pose.hpp"
#include "modular_slam/sensors/camera_frame.hpp"

namespace modular_slam::frontend
{
template <typename F>
concept IsFrontend = requires(F frontend, typename F::InputData data, Pose initialPose) {
    { frontend.process(data, initialPose) } -> std::same_as<typename F::OutputFactor>;
};

template <typename F>
concept IsVisualFrontend = IsFrontend<F> && IsDerivedFromTemplate<typename F::InputData, CameraFrame>;
} // namespace modular_slam::frontend

#endif // MODULAR_SLAM_VISUAL_FRONTEND_HPP_
