#include "modular_slam/frontend/rgbd_feature_frontend.hpp"
#include "modular_slam/basic_types.hpp"
#include "modular_slam/camera.hpp"
#include "modular_slam/camera_parameters.hpp"
#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/depth_frame.hpp"
#include "modular_slam/feature_interface.hpp"
#include "modular_slam/keyframe.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/observation.hpp"
#include "modular_slam/orb_feature.hpp"
#include "modular_slam/parameters_handler.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/slam3d_types.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <boost/range/combine.hpp>
#include <cstdint>
#include <memory>
#include <optional>

namespace mslam
{

Eigen::Vector2f projectOnImage(const Vector3& point, const CameraParameters& cameraParams)
{
    const auto z = point.z();

    return (point.block<2, 1>(0, 0).array() / z * cameraParams.focal.array() + cameraParams.principalPoint.array());
}

bool isVisibleInFrame(const Vector3& point, const CameraParameters& cameraParams, const Size& resolution)
{
    const auto imagePoint = projectOnImage(point, cameraParams);
    const auto inWidth = imagePoint.x() >= 0 && imagePoint.x() < resolution.width;
    const auto inHeight = imagePoint.y() >= 0 && imagePoint.y() < resolution.height;

    return inWidth && inHeight;
}

Vector3 transform(const Vector3& point, const slam3d::SensorState& sensorPose)
{
    Vector3 sensorPoint;

    return sensorPose.orientation.inverse() * point - sensorPoint;
}

bool RgbdFeatureFrontend::init()
{
    using ParamsDefinitionContainer = std::array<std::pair<ParameterDefinition, ParameterValue>, 2>;
    constexpr auto make_param = std::make_pair<ParameterDefinition, ParameterValue>;

    const ParamsDefinitionContainer params = {
        make_param({"rgbd_feature_frontend/min_matched_points", ParameterType::Number, {}, {0, 100000, 1}}, 10.f),
        make_param({"rgbd_feature_frontend/new_keyframe_min_landmarks", ParameterType::Number, {}, {0, 100000, 1}},
                   10.f)};

    for(const auto& [definition, value] : params)
        parametersHandler->registerParameter(definition, value);

    return true;
}

class BasicConstraints : public ConstraintsInterface<slam3d::SensorState, Vector3>
{
  public:
    void addConstraint(const LandmarkConstraint<slam3d::SensorState, Vector3> constraint) override;
    void addConstraint(const KeyframeConstraint<slam3d::SensorState, Vector3> constraint) override;

    void visitLandmarkConstraints(LandmarkConstraintVisitor<slam3d::SensorState, Vector3>& visitor) override;
    void visitKeyframeConstraints(KeyframeConstraintVisitor<slam3d::SensorState, Vector3>& visitor) override;

  private:
    std::vector<LandmarkConstraint<slam3d::SensorState, Vector3>> landmarkConstraints;
    std::vector<KeyframeConstraint<slam3d::SensorState, Vector3>> keyframeConstraints;
};

void BasicConstraints::addConstraint(const LandmarkConstraint<slam3d::SensorState, Vector3> constraint)
{
    landmarkConstraints.push_back(constraint);
}

void BasicConstraints::addConstraint(const KeyframeConstraint<slam3d::SensorState, Vector3> constraint)
{
    keyframeConstraints.push_back(constraint);
}

void BasicConstraints::visitLandmarkConstraints(LandmarkConstraintVisitor<slam3d::SensorState, Vector3>& visitor)
{
    for(const auto& constraint : landmarkConstraints)
        visitor.visit(constraint);
}

void BasicConstraints::visitKeyframeConstraints(KeyframeConstraintVisitor<slam3d::SensorState, Vector3>& visitor)
{
    for(const auto& constraint : keyframeConstraints)
        visitor.visit(constraint);
}

std::optional<Vector3> reconstructPoint(const Eigen::Vector2i imgPoint, const std::uint16_t depth,
                                        const Eigen::Vector2f& principalPoint, const Eigen::Vector2f& invFocal)
{
    const auto isValid = isDepthValid(depth);

    if(!isValid)
        return std::nullopt;

    static constexpr auto M_PER_MM = 0.001f;
    const float z = depth * M_PER_MM;
    const auto x = (imgPoint.x() - principalPoint.x()) * z * invFocal.x();
    const auto y = (imgPoint.y() - principalPoint.y()) * z * invFocal.y();

    return Vector3{x, y, z};
}

/*!
 * \brief Calculates 3D points in camera coordinate system based on points in image coordinate system
 */
std::vector<std::optional<Vector3>>
pointsFromRgbdKeypoints(const DepthFrame& depthFrame,
                        const std::vector<KeypointLandmarkMatch<Eigen::Vector2f, Eigen::Vector3f>>& matches)
{
    std::vector<std::optional<Vector3>> points;
    points.reserve(matches.size());

    const Eigen::Vector2f invFocal = 1.0 / depthFrame.cameraParameters.focal.array();

    for(const auto& match : matches)
    {
        const auto imgPoint = match.match.matchedKeypoint.coordinates.cast<int>();
        const auto depth = getDepth(depthFrame, imgPoint);

        const auto point = reconstructPoint(imgPoint, depth, depthFrame.cameraParameters.principalPoint, invFocal);

        points.push_back(point);
    }

    return points;
}

RgbdFeatureFrontend::RgbdFeatureFrontend(std::shared_ptr<Tracker<slam3d::SensorState, Vector3>> newTracker,
                                         std::shared_ptr<FeatureDetectorInterface<RgbFrame>> newFeatureDetector)
{
    this->tracker = newTracker;
    this->featureDetector = newFeatureDetector;
    constraints = std::make_shared<BasicConstraints>();
}

bool RgbdFeatureFrontend::isNewKeyframeRequired(const int matchedLandmarks) const
{
    const auto minMatchedLandmarks =
        std::get<float>(parametersHandler->getParameter("rgbd_feature_frontend/new_keyframe_min_landmarks").value());

    return matchedLandmarks < minMatchedLandmarks;
}

bool RgbdFeatureFrontend::isLoopClosureNeeded() const
{
    return false;
}

std::shared_ptr<Keyframe<slam3d::SensorState>>
RgbdFeatureFrontend::addKeyframe(const RgbdFrame& sensorData, const slam3d::SensorState& pose,
                                 FeatureInterface<Eigen::Vector2f>& features)
{
    auto newKeyframe = std::make_shared<Keyframe<slam3d::SensorState>>();
    newKeyframe->state = pose;

    const Eigen::Vector2f invFocal = 1.0 / sensorData.depth.cameraParameters.focal.array();

    for(const auto& keypoint : features.getKeypoints())
    {
        const auto keypointCoordinates = keypoint.coordinates.cast<int>();
        const auto landmarkCoordinates =
            reconstructPoint(keypointCoordinates, getDepth(sensorData.depth, keypointCoordinates),
                             sensorData.depth.cameraParameters.principalPoint, invFocal);

        if(landmarkCoordinates.has_value())
        {
            auto landmark = std::make_shared<Landmark<Vector3>>();
            landmark->state = landmarkCoordinates.value();

            features.bindLandmark(keypoint, landmark);
        }
    }

    return newKeyframe;
}

std::shared_ptr<RgbdFeatureFrontend::Constraints> RgbdFeatureFrontend::prepareConstraints(const RgbdFrame& sensorData)
{
    auto features = featureDetector->detect(sensorData.rgb);
    const auto keypoints = features->getKeypoints();

    if(!isInitialized())
    {
        slam3d::SensorState pose = {Vector3(), Quaternion{AngleAxis{0.0f, Vector3{0.0f, 0.0f, 1.0f}}}};
        auto keyframe = addKeyframe(sensorData, pose, *features);

        referenceKeyframeData.features = std::move(features);
        referenceKeyframeData.sensorData = sensorData;
        referenceKeyframeData.keyframe = std::move(keyframe);

        return constraints;
    }

    const auto matchedLandmarks = referenceKeyframeData.features->matchLandmarks(*features);
    constexpr int MIN_POINTS_THRESHOLD = 10;

    if(matchedLandmarks.size() < MIN_POINTS_THRESHOLD)
        return constraints;

    auto points = pointsFromRgbdKeypoints(sensorData.depth, matchedLandmarks);

    std::vector<Vector3> cameraPointsForTracking;
    std::vector<std::shared_ptr<Landmark<Vector3>>> landmarksForTracking;

    for(const auto& [point, matchedLandmark] : boost::combine(points, matchedLandmarks))
    {
        if(point && matchedLandmark.landmark != nullptr)
        {
            cameraPointsForTracking.push_back(point.value());
            landmarksForTracking.push_back(matchedLandmark.landmark);
        }
    }

    const auto pointsMatchedCount = cameraPointsForTracking.size();

    if(pointsMatchedCount > MIN_POINTS_THRESHOLD)
    {
        auto pose = tracker->track(landmarksForTracking, cameraPointsForTracking);

        if(!pose)
            return nullptr;

        referenceKeyframeData.currentPose = pose.value();

        if(isBetterKeyframeNeeded())
        {
            auto bestReferenceKeyframe = findBestKeyframe();
            if(bestReferenceKeyframe != nullptr)
                referenceKeyframeData.keyframe = bestReferenceKeyframe;
        }

        if(isNewKeyframeRequired(pointsMatchedCount))
        {
            auto newKeyframe = std::make_shared<Keyframe<slam3d::SensorState>>();
            newKeyframe->state = pose.value();
            referenceKeyframeData.keyframe = std::move(newKeyframe);

            for(std::size_t i = 0; i < pointsMatchedCount; ++i)
            {
                const auto& match = matchedLandmarks[i];
                const auto& point = cameraPointsForTracking[i];

                LandmarkConstraint<slam3d::SensorState, Vector3> landmarkConstraint{newKeyframe, match.landmark, point};
                constraints->addConstraint(landmarkConstraint);
            }

            // Other landmarks?!
            // findMapLandmarks(); // TODO:

            std::vector<std::shared_ptr<Landmark<Vector3>>> localLandmarks;

            for(const auto& landmark : localLandmarks)
            {
                Vector3 point = transform(landmark->state, pose.value());

                if(isVisibleInFrame(landmark->state, sensorData.depth.cameraParameters, sensorData.depth.size))
                {
                    LandmarkConstraint<slam3d::SensorState, Vector3> landmarkConstraint{newKeyframe, landmark, point};

                    constraints->addConstraint(landmarkConstraint);
                }
            }
        }

        if(isLoopClosureNeeded())
        {
            // auto keyframeLoopClosure->detect();
            // TODO: run loop closure  detection
        }
    }
    else
    {
        // Relocalization

        auto newReferenceKeyframe = relocalize();

        if(newReferenceKeyframe)
        {
            referenceKeyframeData.keyframe = std::move(newReferenceKeyframe);
        }

        return nullptr;
    }

    return constraints;
}

std::shared_ptr<Keyframe<slam3d::SensorState>> RgbdFeatureFrontend::relocalize()
{

    return nullptr;
}

bool RgbdFeatureFrontend::isBetterKeyframeNeeded() const
{
    return false;
}

std::shared_ptr<Keyframe<slam3d::SensorState>> RgbdFeatureFrontend::findBestKeyframe() const
{
    return nullptr;
}

} // namespace mslam
