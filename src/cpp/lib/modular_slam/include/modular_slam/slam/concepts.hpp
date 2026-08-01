#ifndef MODULAR_SLAM_CONCEPTS_HPP_
#define MODULAR_SLAM_CONCEPTS_HPP_

#include "modular_slam/core/constraint.hpp"
#include "modular_slam/core/pose.hpp"
#include "modular_slam/core/time_stamp.hpp"
#include <concepts>
#include <optional>

namespace mslam
{

/**
 * @brief Concept for SLAM Propagator.
 * Responsible for state prediction and state correction.
 */
template <typename T>
concept IsPropagator = requires(T propagator, Timestamp timestamp, Pose state) {
    { propagator.predict(timestamp) } -> std::same_as<Pose>;
    { propagator.correct(state) } -> std::same_as<void>;
};

/**
 * @brief Concept for SLAM Backend.
 * Responsible for constraint management and global optimization.
 */
template <typename T>
concept IsBackend = requires(T backend, Constraint constraint) {
    { backend.addConstraint(constraint) } -> std::same_as<void>;
    { backend.optimize() } -> std::same_as<void>;
    { backend.getState() } -> std::same_as<Pose>;
    { backend.shouldOptimize() } -> std::same_as<bool>;
};

/**
 * @brief Concept for SLAM Map.
 * Responsible for storing and querying spatial data.
 */
template <typename T>
concept IsMap = requires(T map, Pose state) {
    { map.update(state) } -> std::same_as<void>;
};

/**
 * @brief Concept for SLAM Frontend.
 * Responsible for processing sensor data into any optional constraint.
 */
template <typename T>
concept IsFrontend = requires(T frontend, typename T::InputData data, Pose prediction) {
    { frontend.process(data, prediction) } -> std::same_as<std::optional<Constraint>>;
};

} // namespace mslam

#endif // MODULAR_SLAM_CONCEPTS_HPP_
