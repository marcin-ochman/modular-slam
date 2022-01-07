#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <type_traits>

namespace mslam
{

constexpr bool gpu_acceleration = false;

using Frame = std::conditional_t<gpu_acceleration, cv::UMat, cv::Mat>;

template <typename T = float>
using Vector3 = Eigen::Matrix<T, 3, 1>;

template <typename T = float>
using Quaternion = Eigen::Quaternion<T>;

template <typename T = float>
using Transform3 = Eigen::Transform<T, 3, Eigen::Affine>;

} // namespace mslam

#endif /* TYPES_HPP_ */
