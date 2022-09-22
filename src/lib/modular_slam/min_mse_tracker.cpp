#include "modular_slam/min_mse_tracker.hpp"
#include "modular_slam/basic_types.hpp"

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>

namespace mslam
{

struct PointAlignmentCostFunctor
{
    PointAlignmentCostFunctor(std::shared_ptr<Landmark<Vector3>> newLandmark, const Vector3& newLandmarkInCameraCoords)
        : landmark{newLandmark}, landmarkInCameraCoords{newLandmarkInCameraCoords}
    {
    }

    template <typename T>
    bool operator()(const T* const sensorState, T* residual) const
    {
        T p[3];
        T landmarkPoint[3] = {T(landmark->state.x()), T(landmark->state.y()), T(landmark->state.z())};
        ceres::AngleAxisRotatePoint(sensorState, landmarkPoint, p);

        p[0] += sensorState[3];
        p[1] += sensorState[4];
        p[2] += sensorState[5];

        residual[0] = p[0] - T(landmarkInCameraCoords.x());
        residual[1] = p[1] - T(landmarkInCameraCoords.y());
        residual[2] = p[2] - T(landmarkInCameraCoords.z());

        return true;
    }

    std::shared_ptr<Landmark<Vector3>> landmark;
    Vector3 landmarkInCameraCoords;

    static ceres::CostFunction* Create(std::shared_ptr<Landmark<Vector3>> newLandmark,
                                       const Vector3& newLandmarkInCameraCoords)
    {
        return new ceres::AutoDiffCostFunction<PointAlignmentCostFunctor, 3, 6>(
            new PointAlignmentCostFunctor(newLandmark, newLandmarkInCameraCoords));
    }
};

std::optional<slam3d::SensorState>
MinMseTracker::track(const std::vector<std::shared_ptr<Landmark<Vector3>>>& landmarks,
                     const std::vector<Vector3>& sensorPoints)
{
    assert(landmarks.size() == sensorPoints.size());
    ceres::Problem problem;

    double axisAnglePoint[6] = {0, 0, 1, 0, 0, 0};

    for(auto i = 0; i < landmarks.size(); i++)
    {
        auto costFunction = PointAlignmentCostFunctor::Create(landmarks[i], sensorPoints[i]);
        problem.AddResidualBlock(costFunction, nullptr, axisAnglePoint);
    }

    ceres::Solver::Options options;

    options.gradient_tolerance = 1e-8;
    options.function_tolerance = 1e-8;
    options.parameter_tolerance = 1e-8;

    ceres::Solver::Summary summary;

    ceres::Solve(options, &problem, &summary);

    if(!summary.IsSolutionUsable())
        return std::nullopt;

    const float angle = std::sqrt(axisAnglePoint[0] * axisAnglePoint[0] + axisAnglePoint[1] * axisAnglePoint[1] +
                                  axisAnglePoint[2] * axisAnglePoint[2]);
    Vector3 axis = {static_cast<float>(axisAnglePoint[0]), static_cast<float>(axisAnglePoint[1]),
                    static_cast<float>(axisAnglePoint[2])};
    axis /= angle;

    slam3d::SensorState result;
    result.position = {static_cast<float>(axisAnglePoint[3]), static_cast<float>(axisAnglePoint[4]),
                       static_cast<float>(axisAnglePoint[5])};
    result.orientation = Quaternion{AngleAxis(angle, axis)};

    return result;
}

} // namespace mslam
