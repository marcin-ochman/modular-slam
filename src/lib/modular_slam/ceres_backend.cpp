#include "modular_slam/ceres_backend.hpp"
#include "modular_slam/basic_types.hpp"
#include "modular_slam/camera_parameters.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/map_interface.hpp"
#include "modular_slam/rgbd_slam_types.hpp"
#include "modular_slam/slam3d_types.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <boost/polymorphic_cast.hpp>
#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <ceres/types.h>
#include <limits>
#include <optional>
#include <spdlog/spdlog.h>
#include <type_traits>
#include <utility>

namespace mslam
{

struct ReprojectionError
{
    ReprojectionError(const rgbd::RgbdKeypoint& keypoint, const mslam::CameraParameters& cameraParameters)
        : cameraParams(cameraParameters)
    {
        observation.x() = (keypoint.keypoint.coordinates.x() - cameraParams.principalPoint.x()) * keypoint.depth /
                          cameraParams.focal.x();
        observation.y() = (keypoint.keypoint.coordinates.y() - cameraParams.principalPoint.y()) * keypoint.depth /
                          cameraParams.focal.y();
        observation.z() = keypoint.depth;
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

        Eigen::Map<Eigen::Matrix<T, 3, 1>> residualsMapped(residuals);
        residualsMapped = landmarkInCameraCoordinates - observation.cast<T>();

        return true;
    }

    static ceres::CostFunction* Create(const rgbd::RgbdKeypoint& keypoint, const CameraParameters& cameraParameters)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 3, 4, 3, 3>(
            new ReprojectionError(keypoint, cameraParameters)));
    }

    Vector3 observation;
    const mslam::CameraParameters& cameraParams;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ObservationWithCostFunction
{
    LandmarkKeyframeObservation<slam3d::SensorState, Vector3, rgbd::RgbdKeypoint> observation;
    ceres::CostFunction* costFunction;
};

class CeresVisitor : public IMapVisitor<slam3d::SensorState, Vector3, rgbd::RgbdKeypoint>
{
  public:
    explicit CeresVisitor(CameraParameters& newCameraParams) : cameraParams(newCameraParams) {}
    void visit(const LandmarkKeyframeObservation<slam3d::SensorState, Vector3, rgbd::RgbdKeypoint>&
                   landmarkKeyframeObservation) override;

    ceres::Problem& getProblem() { return problem; }
    [[nodiscard]] const std::vector<ObservationWithCostFunction>& getObservationsAndCosts() const
    {
        return observationsWithCosts;
    }

    std::map<Id, int> landmarksCount;
    std::map<Id, int> keyframesCount;

  private:
    ceres::Manifold* quaternionManifold = new ceres::EigenQuaternionManifold;
    ceres::Problem problem;
    const CameraParameters& cameraParams;

    std::vector<ObservationWithCostFunction> observationsWithCosts;
};

CeresBackend::BackendOutputType
CeresBackend::process(FrontendOutput<mslam::slam3d::SensorState, mslam::Vector3, rgbd::RgbdKeypoint>& frontendOutput)
{
    return BackendOutputType();
    if(needsGlobalBundleAdjustment(frontendOutput))
    {
        return globalBundleAdjustment(frontendOutput);
    }
    else if(needsLocalBundleAdjustment(frontendOutput))
    {
        return localBundleAdjustment(frontendOutput);
    }

    return BackendOutputType();
}

bool CeresBackend::init()
{
    using ParamsDefinitionContainer = std::array<std::pair<ParameterDefinition, ParameterValue>, 1>;
    constexpr auto make_param = std::make_pair<ParameterDefinition, ParameterValue>;

    const ParamsDefinitionContainer params = {
        make_param({"ceres_backend/lba_max_num_iterations", ParameterType::Number, {}, {0, 10000, 1}}, 100.f)};

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

void CeresVisitor::visit(
    const LandmarkKeyframeObservation<slam3d::SensorState, Vector3, rgbd::RgbdKeypoint>& observation)
{
    ceres::LossFunction* lossFunction = nullptr;
    ceres::CostFunction* costFunction = ReprojectionError::Create(observation.observation, cameraParams);

    landmarksCount[observation.landmark->id] += 1;
    keyframesCount[observation.keyframe->id] += 1;

    observationsWithCosts.push_back({observation, costFunction});

    problem.AddResidualBlock(costFunction, lossFunction, observation.keyframe->state.orientation.coeffs().data(),
                             observation.keyframe->state.position.data(), observation.landmark->state.data());
    problem.SetManifold(observation.keyframe->state.orientation.coeffs().data(), quaternionManifold);

    if(observation.keyframe->id == 1)
    {
        problem.SetParameterBlockConstant(observation.keyframe->state.orientation.coeffs().data());
        problem.SetParameterBlockConstant(observation.keyframe->state.position.data());
    }
}

CeresBackend::BackendOutputType CeresBackend::localBundleAdjustment(const FrontendOutputType& frontendOutput)
{
    MapVisitingParams visitingParams;
    visitingParams.elementsToVisit = MapElementsToVisit::LandmarkKeyframeObservation;
    visitingParams.landmarkKeyframeObservationParams.graphParams = std::make_optional<GraphBasedParams>();
    visitingParams.landmarkKeyframeObservationParams.graphParams->refKeyframeId = frontendOutput.newKeyframe->id;
    visitingParams.landmarkKeyframeObservationParams.graphParams->deepLevel = 1;

    return bundleAdjustment(visitingParams);
}

CeresBackend::BackendOutputType CeresBackend::globalBundleAdjustment(const FrontendOutputType& frontendOutput)
{
    MapVisitingParams visitingParams;

    visitingParams.elementsToVisit = MapElementsToVisit::LandmarkKeyframeObservation;
    visitingParams.landmarkKeyframeObservationParams.graphParams = std::make_optional<GraphBasedParams>();
    visitingParams.landmarkKeyframeObservationParams.graphParams->refKeyframeId = frontendOutput.newKeyframe->id;
    visitingParams.landmarkKeyframeObservationParams.graphParams->deepLevel = std::numeric_limits<Id>::max();

    return bundleAdjustment(visitingParams);
}

CeresBackend::BackendOutputType CeresBackend::bundleAdjustment(const MapVisitingParams& visitingParams)
{
    // return CeresBackend::BackendOutputType();
    CeresVisitor visitor(cameraParameters);
    map->visit(visitor, visitingParams);

    auto& problem = visitor.getProblem();

    ceres::Solver::Options options;
    options.max_num_iterations = lbaMaxIterations();
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    spdlog::debug("Performing BA {} \n\n {}", problem.NumResidualBlocks(), summary.FullReport());

    return createOutput(visitor);
}

CeresBackend::BackendOutputType CeresBackend::createOutput(const CeresVisitor& visitor) const
{
    BackendOutputType output;
    const auto& allObservations = visitor.getObservationsAndCosts();

    std::unordered_set<std::shared_ptr<slam3d::Keyframe>> keyframes;
    std::unordered_set<std::shared_ptr<slam3d::Landmark>> landmarks;

    constexpr auto squaredErrorThreshold = 0.15 * 0.15;

    for(const auto& [observation, costFunction] : allObservations)
    {
        keyframes.insert(observation.keyframe);
        landmarks.insert(observation.landmark);

        std::array<const double*, 3> parameters = {observation.keyframe->state.orientation.coeffs().data(),
                                                   observation.keyframe->state.position.data(),
                                                   observation.landmark->state.data()};

        Vector3 residuals = Vector3::Zero();
        costFunction->Evaluate(parameters.data(), residuals.data(), nullptr);

        const auto error = residuals.squaredNorm();
        if(error > squaredErrorThreshold)
        {
            output.outlierObservations.push_back(observation);
        }
    }

    output.updatedKeyframes = std::move(keyframes);
    output.updatedLandmarks = std::move(landmarks);

    // if(!output.outlierObservations.empty())
    //     spdlog::error("Outlier found! {}", output.outlierObservations.size());

    return output;
}

} // namespace mslam
