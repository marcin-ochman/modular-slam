#ifndef MODULAR_SLAM_POSE_HPP_
#define MODULAR_SLAM_POSE_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utility>

namespace modular_slam
{

class Pose
{
  private:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d mPosition;
    Eigen::Quaterniond mRotation;

  public:
    Pose() : mPosition(Eigen::Vector3d::Zero()), mRotation(Eigen::Quaterniond::Identity()) {}

    explicit Pose(const Eigen::Matrix4d& T) : Pose(T.col(3).head<3>(), Eigen::Quaterniond(T.block<3, 3>(0, 0))) {}
    Pose(Eigen::Vector3d t, const Eigen::Quaterniond& q) : mPosition(std::move(t)), mRotation(q.normalized()) {}

    Pose operator*(const Pose& other) const
    {
        return Pose(mPosition + mRotation * other.mPosition, mRotation * other.mRotation);
    }

    Pose inverse() const
    {
        const Eigen::Quaterniond q_inv = mRotation.conjugate();
        return Pose(-(q_inv * mPosition), q_inv);
    }

    Eigen::Vector3d transform(const Eigen::Vector3d& p) const { return mPosition + mRotation * p; }

    Eigen::Matrix4d matrix() const
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = mRotation.toRotationMatrix();
        T.block<3, 1>(0, 3) = mPosition;
        return T;
    }

    [[nodiscard]] Eigen::Vector3d position() const { return mPosition; }
    [[nodiscard]] Eigen::Quaterniond quaternion() const { return mRotation; }
};

} // namespace modular_slam

#endif // MODULAR_SLAM_POSE_HPP_
