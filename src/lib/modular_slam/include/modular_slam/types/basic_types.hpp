#ifndef MSLAM_BASIC_TYPES_HPP_
#define MSLAM_BASIC_TYPES_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <cstdint>

namespace mslam
{

using Id = std::uint64_t;

using Vector2 = Eigen::Vector2d;
using Vector2i = Eigen::Vector2i;
using Vector3 = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;
using AngleAxis = Eigen::AngleAxisd;

using Matrix3 = Eigen::Matrix3d;
using Matrix4 = Eigen::Matrix4d;

struct Size
{
    int width;
    int height;
};

} // namespace mslam

#endif // MSLAM_BASIC_TYPES_HPP_
