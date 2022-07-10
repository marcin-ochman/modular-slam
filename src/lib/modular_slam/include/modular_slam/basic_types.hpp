#ifndef MSLAM_BASIC_TYPES_HPP_
#define MSLAM_BASIC_TYPES_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstdint>

#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

namespace mslam
{

using Id = std::uint64_t;

using Vector2d = Eigen::Vector2f;
using Vector3d = Eigen::Vector3f;
using Quaternion = Eigen::Quaternionf;

} // namespace mslam

#endif // MSLAM_BASIC_TYPES_HPP_
