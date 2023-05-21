#include "modular_slam/ceres_reprojection_error_pnp.hpp"
#include "modular_slam/basic_types.hpp"
#include "modular_slam/camera_parameters.hpp"
#include "modular_slam/cv_ransac_pnp.hpp"
#include "modular_slam/slam3d_types.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <boost/range/combine.hpp>
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>

#include <optional>
#include <spdlog/spdlog.h>

namespace mslam
{

class ReprojectionErrorFunctor
{
  public:
    ReprojectionErrorFunctor(Vector3 newPoint, Vector2 newObserved, const CameraParameters& cameraParams)
        : point(newPoint), observed(newObserved), camera{cameraParams}
    {
    }

    template <typename T>
    bool operator()(const T* const sensorState, T* residuals) const
    {
        T pt1[3] = {T(point.x()), T(point.y()), T(point.z())};

        const T* camera_r = &sensorState[0];
        const T* camera_t = &sensorState[3];

        T pt2[3];
        ceres::AngleAxisRotatePoint(camera_r, pt1, pt2);

        pt2[0] = pt2[0] + camera_t[0];
        pt2[1] = pt2[1] + camera_t[1];
        pt2[2] = pt2[2] + camera_t[2];

        const T xp = T(camera.focal.x()) * (pt2[0] / pt2[2]) + T(camera.principalPoint.x());
        const T yp = T(camera.focal.y()) * (pt2[1] / pt2[2]) + T(camera.principalPoint.y());

        const T u = T(observed.x());
        const T v = T(observed.y());

        residuals[0] = u - xp;
        residuals[1] = v - yp;

        return true;
    }

    static ceres::CostFunction* Create(Vector3 points, Vector2 observed, const CameraParameters& cameraParams)
    {
        return new ceres::AutoDiffCostFunction<ReprojectionErrorFunctor, 2, 6>(
            new ReprojectionErrorFunctor(points, observed, cameraParams));
    }

  private:
    Vector3 point;
    Vector2 observed;
    const CameraParameters& camera;
};

std::optional<MinMseTracker::PnpResult>
MinMseTracker::solvePnp(const std::vector<std::shared_ptr<Landmark<Vector3>>>& landmarks,
                        const std::vector<Vector2>& sensorPoints, const slam3d::SensorState& initial)
{
    assert(landmarks.size() == sensorPoints.size());
    ceres::Problem problem;
    Eigen::AngleAxisd angleAxis;
    angleAxis = initial.orientation.cast<double>();
    auto axis = angleAxis.axis() * angleAxis.angle();
    double axisAnglePoint[6] = {
        axis.x(), axis.y(), axis.z(), initial.position.x(), initial.position.y(), initial.position.z()};

    spdlog::debug("camera params f: ({},{}), c: ({},{})", cameraParams.focal.x(), cameraParams.focal.y(),
                  cameraParams.principalPoint.x(), cameraParams.principalPoint.y());

    for(const auto& [landmark, point] : boost::combine(landmarks, sensorPoints))
    {
        auto costFunction = ReprojectionErrorFunctor::Create(landmark->state, point, cameraParams);
        problem.AddResidualBlock(costFunction, nullptr, axisAnglePoint);
    }

    ceres::Solver::Options options;

    options.gradient_tolerance = 1e-8;
    options.function_tolerance = 1e-8;
    options.parameter_tolerance = 1e-8;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if(!summary.IsSolutionUsable())
        return std::nullopt;

    const float resultAngle =
        static_cast<float>(std::sqrt(axisAnglePoint[0] * axisAnglePoint[0] + axisAnglePoint[1] * axisAnglePoint[1] +
                                     axisAnglePoint[2] * axisAnglePoint[2]));
    Vector3 resultAxis = {static_cast<float>(axisAnglePoint[0]), static_cast<float>(axisAnglePoint[1]),
                          static_cast<float>(axisAnglePoint[2])};
    resultAxis /= resultAngle;

    PnpResult result;
    result.pose.position = {static_cast<float>(axisAnglePoint[3]), static_cast<float>(axisAnglePoint[4]),
                            static_cast<float>(axisAnglePoint[5])};
    result.pose.orientation = Quaternion{AngleAxis(resultAngle, resultAxis)};

    return result;
}

} // namespace mslam
