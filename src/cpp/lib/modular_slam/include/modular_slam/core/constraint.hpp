#ifndef MODULAR_SLAM_CONSTRAINT_HPP_
#define MODULAR_SLAM_CONSTRAINT_HPP_

#include <Eigen/Dense>
#include <cstdint>
#include <utility>
#include <vector>

namespace mslam
{

/**
 * @brief A generic representation of a SLAM constraint (factor).
 *
 * This structure is designed to be agnostic to the specific type of SLAM
 * (Feature, Direct, ML), serving as the primary communication medium
 * between Frontends and the Backend.
 */
struct Constraint
{
    // The indices of the states/landmarks involved in this constraint
    // e.g., {pose_id, pose_id} for a relative pose constraint
    std::vector<std::uint64_t> indices;

    // The observed value (measurement)
    Eigen::VectorXd measurement;

    // The uncertainty/weight of the measurement (information matrix)
    Eigen::MatrixXd information;

    // A type identifier used by the Backend to determine which
    // error function to apply.
    std::uint32_t type;

    Constraint() = default;
    Constraint(std::vector<std::uint64_t> idx, Eigen::VectorXd meas, Eigen::MatrixXd info, std::uint32_t t)
        : indices(std::move(idx)), measurement(std::move(meas)), information(std::move(info)), type(t)
    {
    }
};

} // namespace mslam

#endif // MODULAR_SLAM_CONSTRAINT_HPP_
