#ifndef MODULAR_SLAM_CAMERA_FRAME_HPP_
#define MODULAR_SLAM_CAMERA_FRAME_HPP_

#include "modular_slam/core/time_stamp.hpp"
#include "modular_slam/sensors/camera/camera_concepts.hpp"
namespace modular_slam
{

struct Image
{
};

struct AcquisitionInfo
{
    uint64_t id;
    Timestamp timestamp;
};

template <IsCamera CameraPolicy>
struct CameraFrame
{
    using Scalar = typename CameraPolicy::Scalar;

    AcquisitionInfo acquisition;
    CameraPolicy sensor;
    Image image;

    auto project(const Vec3<Scalar>& point3d) const { return sensor.project(point3d); }
    auto unproject(const Vec2<Scalar>& uv, Scalar depth) const { return sensor.unproject(uv, depth); }
};
} // namespace modular_slam

#endif // MODULAR_SLAM_CAMERA_FRAME_HPP_
