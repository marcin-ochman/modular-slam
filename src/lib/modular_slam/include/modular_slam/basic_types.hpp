#ifndef MSLAM_BASIC_TYPES_HPP_
#define MSLAM_BASIC_TYPES_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <cstdint>

#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

namespace mslam
{

using Id = std::uint64_t;

using Vector2 = Eigen::Vector2f;
using Vector2i = Eigen::Vector2i;
using Vector3 = Eigen::Vector3f;
using Quaternion = Eigen::Quaternionf;
using AngleAxis = Eigen::AngleAxisf;

struct Size
{
    int width;
    int height;
};

} // namespace mslam

#endif // MSLAM_BASIC_TYPES_HPP_
