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
#include <algorithm>

#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/uniqued.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/combine.hpp>

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <optional>
#include <spdlog/spdlog.h>

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
    using ParamsDefinitionContainer = std::array<std::pair<ParameterDefinition, ParameterValue>, 3>;
    constexpr auto make_param = std::make_pair<ParameterDefinition, ParameterValue>;

    const ParamsDefinitionContainer params = {
        make_param({"rgbd_feature_frontend/min_matched_points", ParameterType::Number, {}, {0, 100000, 1}}, 10.f),
        make_param({"rgbd_feature_frontend/better_keyframe_landmarks", ParameterType::Number, {}, {0, 100000, 1}},
                   60.f),
        make_param({"rgbd_feature_frontend/new_keyframe_min_landmarks", ParameterType::Number, {}, {0, 100000, 1}},
                   30.f)};

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

std::optional<Vector3> reconstructPoint(const Eigen::Vector2i imgPoint, const float depth,
                                        const Eigen::Vector2f& principalPoint, const Eigen::Vector2f& invFocal)
{
    const auto isValid = isDepthValid(depth);

    if(!isValid)
        return std::nullopt;

    const float z = depth;
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
        const Eigen::Vector2i imgPoint = match.match.matchedKeypoint.coordinates.cast<int>();
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

    return newKeyframe;
}

std::shared_ptr<RgbdFeatureFrontend::Constraints> RgbdFeatureFrontend::prepareConstraints(const RgbdFrame& sensorData)
{
    auto features = featureDetector->detect(sensorData.rgb);

    if(!hasInitialKeyframe())
    {
        initFirstKeyframe(sensorData, std::move(features));
        return constraints;
    }

    if(auto trackConstraints = track(sensorData, *features); trackConstraints != nullptr)
    {
        if(isLoopClosureNeeded())
        {
            // auto keyframeLoopClosure->detect();
            // TODO: run loop closure  detection
        }

        // TODO: update constraints
        return constraints;
    }
    else
    {

        if(auto newReferenceKeyframe = relocalize(); newReferenceKeyframe != nullptr)
        {
            referenceKeyframeData.keyframe = std::move(newReferenceKeyframe);

            // TODO: update constraints
            return constraints;
        }

        spdlog::error("Relocalization failed. Tracking lost");

        return nullptr;
    }

    return constraints;
}

std::shared_ptr<RgbdFeatureFrontend::Constraints>
RgbdFeatureFrontend::track(const RgbdFrame& sensorData, FeatureInterface<Eigen::Vector2f>& features)
{
    auto trackConstraints = std::make_shared<BasicConstraints>();
    const auto matchedLandmarks = referenceKeyframeData.features->matchLandmarks(features);
    auto points = pointsFromRgbdKeypoints(sensorData.depth, matchedLandmarks);

    spdlog::info("Points from RGBD keypoints {}, matchedLandmarks: {}", points.size(), matchedLandmarks.size());
    std::vector<Vector3> cameraPointsForTracking;
    std::vector<std::shared_ptr<Landmark<Vector3>>> landmarksForTracking;

    for(const auto& [point, matchedLandmark] : boost::combine(points, matchedLandmarks))
    {
        if(point.has_value() && matchedLandmark.landmark != nullptr)
        {
            cameraPointsForTracking.push_back(point.value());
            landmarksForTracking.push_back(matchedLandmark.landmark);
        }
    }

    const auto pointsMatchedCount = cameraPointsForTracking.size();

    spdlog::info("Tracking points with feference keyframe matched {}", pointsMatchedCount);
    if(pointsMatchedCount < minMatchedPoints())
        return nullptr;

    auto pose = tracker->track(landmarksForTracking, cameraPointsForTracking);

    if(!pose)
        return nullptr;

    referenceKeyframeData.currentPose = pose.value();

    if(isBetterReferenceKeyframeNeeded(pointsMatchedCount))
    {
        auto bestReferenceKeyframe = findBestKeyframe();
        if(bestReferenceKeyframe != nullptr)
        {
            referenceKeyframeData.keyframe = bestReferenceKeyframe;
        }
    }

    if(isNewKeyframeRequired(pointsMatchedCount))
    {
        spdlog::info("New keyframe required");
        auto newKeyframe = std::make_shared<Keyframe<slam3d::SensorState>>();
        newKeyframe->state = pose.value();

        for(const auto& [point, landmark] : boost::combine(cameraPointsForTracking, landmarksForTracking))
        {
            LandmarkConstraint<slam3d::SensorState, Vector3> landmarkConstraint{newKeyframe, landmark, point};
            trackConstraints->addConstraint(landmarkConstraint);
        }

        auto localLandmarks = findVisibleLocalLandmarks(features, pose.value(), sensorData);
        std::vector<std::shared_ptr<Landmark<Vector3>>> landmarksOnFrame;
        auto inserter = std::inserter(landmarksOnFrame, landmarksOnFrame.end());

        const auto newLandmarkKeypoints = findKeypointsForNewLandmarks(features, landmarksOnFrame);
        const Eigen::Vector2f invFocal = 1.0 / sensorData.depth.cameraParameters.focal.array();
        for(const auto& landmarkKeypoint : newLandmarkKeypoints)
        {
            auto landmark = std::make_shared<Landmark<Vector3>>();

            const auto landmarkCoordinatesInKeyframe =
                reconstructPoint(landmarkKeypoint.coordinates, getDepth(sensorData.depth, landmarkKeypoint.coordinates),
                                 sensorData.depth.cameraParameters.principalPoint, invFocal);

            if(landmarkCoordinatesInKeyframe.has_value())
            {
                LandmarkConstraint<slam3d::SensorState, Vector3> landmarkConstraint{
                    newKeyframe, landmark, landmarkCoordinatesInKeyframe.value()};
                trackConstraints->addConstraint(landmarkConstraint);
            }
        }
    }

    return trackConstraints;
}

void RgbdFeatureFrontend::initFirstKeyframe(const RgbdFrame& sensorData,
                                            std::unique_ptr<FeatureInterface<Eigen::Vector2f>> features)
{
    const slam3d::SensorState pose = {Vector3(), Quaternion{AngleAxis{0.0f, Vector3{0.0f, 0.0f, 1.0f}}}};
    auto keyframe = std::make_shared<mslam::slam3d::Keyframe>();
    keyframe->state = pose;
    const Eigen::Vector2f invFocal = 1.0 / sensorData.depth.cameraParameters.focal.array();

    const auto keypoints = features->keypoints();
    const auto descriptors = features->descriptors();
    for(const auto& [keypoint, descriptor] : boost::combine(keypoints, descriptors))
    {
        const auto keypointCoordinates = keypoint.coordinates.cast<int>();
        const auto landmarkCoordinates =
            reconstructPoint(keypointCoordinates, getDepth(sensorData.depth, keypointCoordinates),
                             sensorData.depth.cameraParameters.principalPoint, invFocal);

        if(landmarkCoordinates.has_value())
        {
            auto landmark = std::make_shared<Landmark<Vector3>>();
            landmark->state = landmarkCoordinates.value();
            features->bindLandmark(keypoint, landmark);

            LandmarkConstraint<slam3d::SensorState, Vector3> landmarkConstraint{keyframe, landmark,
                                                                                landmarkCoordinates.value()};
            constraints->addConstraint(landmarkConstraint);

            referenceKeyframeData.landmarkDescriptors.insert(std::make_pair(landmark, descriptor));
        }
    }

    referenceKeyframeData.features = std::move(features);
    referenceKeyframeData.sensorData = sensorData; // TODO: check!
    referenceKeyframeData.keyframe = std::move(keyframe);
}

std::size_t RgbdFeatureFrontend::minMatchedPoints() const
{
    const auto minMatchedLandmarks =
        std::get<float>(parametersHandler->getParameter("rgbd_feature_frontend/min_matched_points").value());

    return static_cast<int>(minMatchedLandmarks);
}

std::vector<std::shared_ptr<Landmark<Vector3>>>
RgbdFeatureFrontend::findVisibleLocalLandmarks(const FeatureInterface<Eigen::Vector2f>& features,
                                               const slam3d::SensorState& pose, const RgbdFrame& sensorData) const
{
    std::vector<std::shared_ptr<Landmark<Vector3>>> localLandmarks;
    std::vector<std::shared_ptr<Keyframe<slam3d::SensorState>>> localKeyframes;
    std::vector<Descriptor> localLandmarksDescriptors;

    boost::copy(boost::adaptors::keys(referenceKeyframeData.landmarkDescriptors) |
                    boost::adaptors::filtered(
                        [&cameraParameters = sensorData.depth.cameraParameters,
                         &imgSize = sensorData.depth.size](const std::shared_ptr<Landmark<Vector3>> landmark)
                        { return isVisibleInFrame(landmark->state, cameraParameters, imgSize); }) |
                    boost::adaptors::uniqued,
                std::back_inserter(localLandmarks));

    std::transform(std::begin(localLandmarks), std::end(localLandmarks), std::back_inserter(localLandmarksDescriptors),
                   [&landmarkDescriptors = referenceKeyframeData.landmarkDescriptors](
                       const std::shared_ptr<Landmark<Eigen::Vector3f>>& landmark)
                   {
                       auto it = landmarkDescriptors.find(landmark);
                       return it->second;
                   });

    auto matches = features.match(localLandmarksDescriptors);
    std::vector<std::shared_ptr<Landmark<Vector3>>> matchedLandmarks;
    matchedLandmarks.reserve(matches.size());

    std::transform(std::begin(matches), std::end(matches), std::back_inserter(matchedLandmarks),
                   [&localLandmarks](const DescriptorMatch& match) { return localLandmarks[match.toIndex]; });
    spdlog::info("found Local landmarks {} \n\n", matches.size());
    return matchedLandmarks;
}

std::vector<Keypoint<Eigen::Vector2i>> RgbdFeatureFrontend::findKeypointsForNewLandmarks(
    const FeatureInterface<Eigen::Vector2f>& features, const boost::span<std::shared_ptr<Landmark<Vector3>>> landmarks

) const
{

    // TODO: implement
    return {};
}

std::shared_ptr<Keyframe<slam3d::SensorState>> RgbdFeatureFrontend::relocalize()
{
    // TODO: implement
    return nullptr;
}

bool RgbdFeatureFrontend::isBetterReferenceKeyframeNeeded(const int keyframeLandmarksCount) const
{
    const auto maxLandmarks =
        std::get<float>(parametersHandler->getParameter("rgbd_feature_frontend/better_keyframe_landmarks").value());

    return keyframeLandmarksCount < maxLandmarks; // TODO: add rest
}

std::shared_ptr<Keyframe<slam3d::SensorState>> RgbdFeatureFrontend::findBestKeyframe() const
{
    // TODO: implement
    return nullptr;
}

} // namespace mslam
