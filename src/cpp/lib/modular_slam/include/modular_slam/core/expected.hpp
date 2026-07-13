#ifndef MODULAR_SLAM_EXPECTED_HPP_
#define MODULAR_SLAM_EXPECTED_HPP_

#include "modular_slam/core/error.hpp"
#include <expected>

namespace mslam
{
template <typename T>
using Expected = std::expected<T, Error>;

using Status = std::expected<void, Error>;

template <typename T>
using ExpectedRef = std::expected<std::reference_wrapper<T>, Error>;

template <typename T>
using ExpectedConstRef = std::expected<std::reference_wrapper<const T>, Error>;

} // namespace mslam

#endif // MODULAR_SLAM_EXPECTED_HPP_
