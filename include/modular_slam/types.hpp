#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <type_traits>

namespace mslam
{

enum class DeviceType
{
    RGB,
    STEREO_RGB,
    RGBD,
    RGBD_IMU
};

constexpr bool gpu_acceleration = false;

using Frame = std::conditional_t<gpu_acceleration, cv::UMat, cv::Mat>;

struct RgbdSensorData
{
    Frame rgb;
    Frame depth;
};

struct ImuSensorData
{
};

struct RgbdImuSensorData
{
    RgbdSensorData rgbd;
    ImuSensorData imu;
};

using Vector3 = Eigen::Matrix<float, 3, 1>;
using Quaternion = Eigen::Quaternion<float>;
using Transform = Eigen::Transform<float, 3, Eigen::Affine>;

} // namespace mslam

#endif /* TYPES_HPP_ */
