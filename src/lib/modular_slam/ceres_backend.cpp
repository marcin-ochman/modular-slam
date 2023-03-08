#include "modular_slam/ceres_backend.hpp"
#include "modular_slam/basic_types.hpp"
#include "modular_slam/camera_parameters.hpp"
#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/map_interface.hpp"
#include "modular_slam/rgbd_slam_types.hpp"
#include "modular_slam/slam3d_types.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <ceres/types.h>
#include <limits>
#include <optional>
#include <spdlog/spdlog.h>

namespace mslam
{

// class PoseGraph3dErrorTerm
// {
//   public:
//     PoseGraph3dErrorTerm(mslam::rgbd::SensorState t_ab_measured, Eigen::Matrix<double, 6, 6> sqrt_information)
//         : t_ab_measured_(std::move(t_ab_measured)), sqrt_information_(std::move(sqrt_information))
//     {
//     }
//     template <typename T>
//     bool operator()(const T* const p_a_ptr, const T* const q_a_ptr, const T* const p_b_ptr, const T* const q_b_ptr,
//                     T* residuals_ptr) const
//     {
//         Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
//         Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);
//         Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
//         Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);
//         // Compute the relative transformation between the two frames.
//         Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
//         Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;
//         // Represent the displacement between the two frames in the A frame.
//         Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);
//         // Compute the error  between the two orientation estimates.
//         Eigen::Quaternion<T> delta_q = t_ab_measured_.orientation.template cast<T>() * q_ab_estimated.conjugate();
//         // Compute the residuals.
//         // [ position         ]   [ delta_p          ]
//         // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
//         Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
//         residuals.template block<3, 1>(0, 0) = p_ab_estimated - t_ab_measured_.position.template cast<T>();
//         residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();
//         // Scale the residuals by the measurement uncertainty.
//         residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

//         return true;
//     }

//     static ceres::CostFunction* Create(const mslam::rgbd::SensorState& t_ab_measured,
//                                        const Eigen::Matrix<double, 6, 6>& sqrt_information)
//     {
//         return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>(
//             new PoseGraph3dErrorTerm(t_ab_measured, sqrt_information));
//     }

//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//   private:
//     // The measurement for the position of B relative to A in the A frame.
//     const mslam::rgbd::SensorState t_ab_measured_;
//     // The square root of the measurement information matrix.
//     const Eigen::Matrix<double, 6, 6> sqrt_information_;
// };

struct ReprojectionError
{
    ReprojectionError(const Vector2& observation, const mslam::CameraParameters& cameraParameters)
        : observed(observation), cameraParams(cameraParameters)
    {
    }

    template <typename T>
    bool operator()(const T* const keyframeOrientation, const T* const position, const T* const landmarkPoint,
                    T* residuals) const
    {
        Eigen::Map<const Eigen::Quaternion<T>> orientationMapped(keyframeOrientation);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> positionMapped(position);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> landmarkPointMapped(landmarkPoint);
        const auto inverse = orientationMapped.inverse();

        const Eigen::Matrix<T, 3, 1> landmarkInCameraCoordinates =
            inverse * landmarkPointMapped - inverse * positionMapped;
        const Eigen::Matrix<T, 2, 1> predicted = cameraParams.focal.array() *
                                                     landmarkInCameraCoordinates.template block<2, 1>(0, 0).array() /
                                                     landmarkInCameraCoordinates.z() +
                                                 cameraParams.principalPoint.array();

        residuals[0] = predicted.x() - T(observed.x());
        residuals[1] = predicted.y() - T(observed.y());

        return true;
    }

    static ceres::CostFunction* Create(const Vector2& keypoint, const CameraParameters& cameraParameters)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3, 3>(
            new ReprojectionError(keypoint, cameraParameters)));
    }

    const Vector2& observed;
    const mslam::CameraParameters& cameraParams;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class CeresVisitor : public IMapVisitor<slam3d::SensorState, Vector3>
{
  public:
    explicit CeresVisitor(CameraParameters& newCameraParams) : cameraParams(newCameraParams) {}
    // void visit(std::shared_ptr<Landmark<slam3d::LandmarkState>> landmark) override;
    // void visit(std::shared_ptr<Keyframe<slam3d::SensorState>> keyframe) override;
    void visit(const LandmarkKeyframeObservation<slam3d::SensorState, Vector3>& landmarkKeyframeObservation) override;

    ceres::Problem& getProblem() { return problem; }

  private:
    ceres::Manifold* quaternionManifold = new ceres::EigenQuaternionManifold;
    ceres::Problem problem;
    const CameraParameters& cameraParams;
};

CeresBackend::BackendOutputType
CeresBackend::process(FrontendOutput<mslam::slam3d::SensorState, mslam::Vector3>& frontendOutput)
{
    BackendOutputType output;
    if(needsGlobalBundleAdjustment(frontendOutput))
    {
        globalBundleAdjustment(frontendOutput);
    }
    else if(needsLocalBundleAdjustment(frontendOutput))
    {
        localBundleAdjustment(frontendOutput);
    }

    return output;
}

bool CeresBackend::init()
{
    using ParamsDefinitionContainer = std::array<std::pair<ParameterDefinition, ParameterValue>, 1>;
    constexpr auto make_param = std::make_pair<ParameterDefinition, ParameterValue>;

    const ParamsDefinitionContainer params = {
        make_param({"ceres_backend/lba_max_num_iterations", ParameterType::Number, {}, {0, 10000, 1}}, 10.f)};

    for(const auto& [definition, value] : params)
    {
        parametersHandler->registerParameter(definition, value);
    }

    return true;
}

[[nodiscard]] int CeresBackend::lbaMaxIterations() const
{
    return static_cast<int>(
        std::get<float>(parametersHandler->getParameter("ceres_backend/lba_max_num_iterations").value()));
}

bool CeresBackend::needsGlobalBundleAdjustment(const FrontendOutputType& frontendOutput) const
{
    return frontendOutput.loopDetected.has_value();
}

bool CeresBackend::needsLocalBundleAdjustment(const FrontendOutputType& frontendOutput) const
{
    return frontendOutput.newKeyframe != nullptr;
}

void CeresVisitor::visit(const LandmarkKeyframeObservation<slam3d::SensorState, Vector3>& observation)
{
    ceres::LossFunction* lossFunction = nullptr;
    ceres::CostFunction* costFunction = ReprojectionError::Create(observation.keypoint.coordinates, cameraParams);

    problem.AddResidualBlock(costFunction, lossFunction, observation.keyframe->state.orientation.coeffs().data(),
                             observation.keyframe->state.position.data(), observation.landmark->state.data());
    problem.SetManifold(observation.keyframe->state.orientation.coeffs().data(), quaternionManifold);

    if(observation.keyframe->id == 0)
    {
        problem.SetParameterBlockConstant(observation.keyframe->state.orientation.coeffs().data());
        problem.SetParameterBlockConstant(observation.keyframe->state.position.data());
    }
}

void CeresBackend::localBundleAdjustment(const FrontendOutputType& frontendOutput)
{
    MapVisitingParams visitingParams;
    visitingParams.elementsToVisit = MapElementsToVisit::LandmarkKeyframeObservation;
    visitingParams.landmarkKeyframeObservationParams.graphParams = std::make_optional<GraphBasedParams>();
    visitingParams.landmarkKeyframeObservationParams.graphParams->refKeyframeId = frontendOutput.newKeyframe->id;
    visitingParams.landmarkKeyframeObservationParams.graphParams->deepLevel = 1;

    bundleAdjustment(visitingParams);
}

void CeresBackend::globalBundleAdjustment(const FrontendOutputType& frontendOutput)
{
    MapVisitingParams visitingParams;
    visitingParams.elementsToVisit = MapElementsToVisit::LandmarkKeyframeObservation;
    visitingParams.landmarkKeyframeObservationParams.graphParams = std::make_optional<GraphBasedParams>();
    visitingParams.landmarkKeyframeObservationParams.graphParams->refKeyframeId = frontendOutput.newKeyframe->id;
    visitingParams.landmarkKeyframeObservationParams.graphParams->deepLevel = std::numeric_limits<Id>::max();

    bundleAdjustment(visitingParams);
}

void CeresBackend::bundleAdjustment(const MapVisitingParams& visitingParams)
{
    CeresVisitor visitor(cameraParameters);
    map->visit(visitor, visitingParams);

    auto& problem = visitor.getProblem();

    ceres::Solver::Options options;
    options.max_num_iterations = lbaMaxIterations();
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    spdlog::info("Performing BA {} \n\n {}", problem.NumResidualBlocks(), summary.FullReport());
}

} // namespace mslam
