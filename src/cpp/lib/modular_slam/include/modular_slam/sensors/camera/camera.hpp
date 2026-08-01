#ifndef MODULAR_SLAM_CAMERA_HPP
#define MODULAR_SLAM_CAMERA_HPP

#include "modular_slam/core/concept_utils.hpp"
#include "modular_slam/core/pose.hpp"
#include "modular_slam/core/time_stamp.hpp"
#include "modular_slam/core/vectors.hpp"
#include "modular_slam/sensors/camera/camera_concepts.hpp"
#include "modular_slam/sensors/camera_frame.hpp"

#include <concepts>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace mslam::sensors::camera
{

class BaseCameraModel;

template <typename T>
concept IsCameraModel = requires(T camera) {
    { camera.width() } -> std::convertible_to<std::size_t>;
    { camera.height() } -> std::convertible_to<std::size_t>;
    { camera.focalLength() } -> std::convertible_to<double>;
    { camera.calibration() } -> std::convertible_to<const BaseCameraModel&>;
};

template <typename T>
concept IsStandardCamera = IsCameraModel<T> && std::derived_from<T, BaseCameraModel>;

class BaseCameraModel
{
  public:
    virtual ~BaseCameraModel() = default;

    virtual std::size_t width() const = 0;
    virtual std::size_t height() const = 0;

  protected:
    double mFocalLength = 1.0;
    double mPrincipalPointX = 0.5;
    double mPrincipalPointY = 0.5;
    std::vector<double> mDistortionCoefficients;

  public:
    double focalLength() const { return mFocalLength; }
    double principalPointX() const { return mPrincipalPointX; }
    double principalPointY() const { return mPrincipalPointY; }
    const std::vector<double>& distortionCoefficients() const { return mDistortionCoefficients; }
    void setFocalLength(double f) { mFocalLength = f; }
    void setPrincipalPoint(double cx, double cy)
    {
        mPrincipalPointX = cx;
        mPrincipalPointY = cy;
    }
    void setDistortionCoefficients(const std::vector<double>& coeffs) { mDistortionCoefficients = coeffs; }
    bool hasDistortion() const { return !mDistortionCoefficients.empty(); }
};

template <typename Derived>
class CameraModel : public BaseCameraModel
{
  public:
    Derived* self() { return static_cast<Derived*>(this); }
    const Derived* self() const { return static_cast<const Derived*>(this); }

    std::string typeName() const { return Derived::staticTypeName(); }
};

class Camera
{
  public:
    Camera() = default;
    virtual ~Camera() = default;

    virtual int id() const = 0;
    virtual BaseCameraModel& model() = 0;
    virtual const BaseCameraModel& model() const = 0;

    virtual Image createImage(const std::vector<std::uint8_t>& data, Timestamp timestamp) = 0;
    virtual bool isValid() const = 0;

    virtual void publishCalibration() const = 0;
};

template <typename Derived>
class BaseCamera : public Camera
{
  protected:
    int mCameraId;
    std::unique_ptr<BaseCameraModel> mModel;

  public:
    explicit BaseCamera(int id, std::unique_ptr<BaseCameraModel> model) : mCameraId(id), mModel(std::move(model)) {}

    int id() const override { return mCameraId; }
    BaseCameraModel& model() override { return *mModel; }
    const BaseCameraModel& model() const override { return *mModel; }

    bool isValid() const override { return mModel && mModel->width() > 0 && mModel->height() > 0; }

    void publishCalibration() const override
    {
        if(mModel)
        {
            // Publish calibration tensor / ROS parameters
        }
    }
};

struct CameraParameters
{
    std::vector<double> intrinsicMatrix;
    std::vector<double> distortionCoefficients;
    std::size_t imageWidth;
    std::size_t imageHeight;
    std::string cameraFrame;
    double baseline = 0.0;
};

class CameraParametersExtractor
{
  public:
    virtual ~CameraParametersExtractor() = default;

    virtual std::optional<CameraParameters> extractFromConfig(const std::string& path) const = 0;
    virtual std::optional<CameraParameters> extractFromExtrinsics(const std::string& path) const = 0;
};

} // namespace mslam::sensors::camera

#endif // MODULAR_SLAM_CAMERA_HPP
