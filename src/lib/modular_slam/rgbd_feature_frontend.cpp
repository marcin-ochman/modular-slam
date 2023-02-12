#include "modular_slam/frontend/rgbd_feature_frontend.hpp"
#include "modular_slam/basic_types.hpp"
#include "modular_slam/camera_parameters.hpp"
#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/depth_frame.hpp"
#include "modular_slam/feature_interface.hpp"
#include "modular_slam/loop_detection.hpp"
#include "modular_slam/orb_feature.hpp"
#include "modular_slam/parameters_handler.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/rgbd_slam_types.hpp"
#include "modular_slam/slam3d_types.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/adaptor/uniqued.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/combine.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <numeric>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace mslam
{

class BasicConstraints : public ConstraintsInterface<slam3d::SensorState, Vector3>
{
  public:
    void addConstraint(const LandmarkObservationConstraint<slam3d::SensorState, Vector3> constraint) override;
    void addConstraint(const KeyframeConstraint<slam3d::SensorState, Vector3> constraint) override;
    void visitConstraints(IConstraintVisitor<slam3d::SensorState, Vector3>& visitor) override;
    bool removeKeyframe(const Keyframe<slam3d::SensorState>& keyframe) override;
    bool removeLandmark(const Landmark<Vector3>& landmark) override;
    slam3d::SensorState currentState() override { return state; }
    void setState(const slam3d::SensorState& newState) { state = newState; }

  private:
    std::vector<LandmarkObservationConstraint<slam3d::SensorState, Vector3>> landmarkConstraints;
    std::vector<KeyframeConstraint<slam3d::SensorState, Vector3>> keyframeConstraints;

    slam3d::SensorState state;
};

Eigen::Vector2f projectOnImage(const Vector3& point, const CameraParameters& cameraParams)
{
    const auto z = point.z();

    return point.block<2, 1>(0, 0).array() / z * cameraParams.focal.array() + cameraParams.principalPoint.array();
}

Vector3 toCameraCoordinates(const Vector3& point, const slam3d::SensorState& sensorPose)
{
    const auto inverse = sensorPose.orientation.inverse();
    return inverse * point - inverse * sensorPose.position;
}

slam3d::SensorState toCameraCoordinateSystemProjection(const slam3d::SensorState& state)
{
    const auto inverse = state.orientation.inverse();

    slam3d::SensorState invState;
    invState.orientation = inverse;
    invState.position = -(inverse * state.position);

    return invState;
}

const slam3d::SensorState& toWorldCoordinateSystemProjection(const slam3d::SensorState& state)
{
    return state;
}

Eigen::Vector2f projectOnImage(const Vector3& point, const CameraParameters& cameraParams,
                               const slam3d::SensorState& sensorPose)
{
    const auto pointCameraCoordinates = toCameraCoordinates(point, sensorPose);
    return projectOnImage(pointCameraCoordinates, cameraParams);
}

bool isVisibleInFrame(const Vector3& point, const CameraParameters& cameraParams, const Size& resolution)
{
    const auto imagePoint = projectOnImage(point, cameraParams);
    const auto inWidth = imagePoint.x() >= 0 && imagePoint.x() < static_cast<float>(resolution.width);
    const auto inHeight = imagePoint.y() >= 0 && imagePoint.y() < static_cast<float>(resolution.height);

    return inWidth && inHeight;
}

Vector3 toGlobalCoordinates(const Vector3& point, const slam3d::SensorState& sensorPose)
{
    return sensorPose.orientation * point + sensorPose.position;
}

bool isVisibleInFrame(const Vector3& point, const slam3d::SensorState& cameraPose, const CameraParameters& cameraParams,
                      const Size& resolution)
{
    auto pointInCameraCoordinates = toGlobalCoordinates(point, cameraPose);

    return isVisibleInFrame(pointInCameraCoordinates, cameraParams, resolution);
}

bool RgbdFeatureFrontend::init()
{
    using ParamsDefinitionContainer = std::array<std::pair<ParameterDefinition, ParameterValue>, 3>;
    constexpr auto make_param = std::make_pair<ParameterDefinition, ParameterValue>;

    const ParamsDefinitionContainer params = {
        make_param({"rgbd_feature_frontend/min_matched_points", ParameterType::Number, {}, {0, 100000, 1}}, 10.f),
        make_param({"rgbd_feature_frontend/better_keyframe_landmarks", ParameterType::Number, {}, {0, 10000, 1}}, 60.f),
        make_param({"rgbd_feature_frontend/new_keyframe_min_landmarks", ParameterType::Number, {}, {0, 10000, 1}},
                   30.f)};

    for(const auto& [definition, value] : params)
    {
        parametersHandler->registerParameter(definition, value);
    }

    return true;
}

void BasicConstraints::addConstraint(const LandmarkObservationConstraint<slam3d::SensorState, Vector3> constraint)
{
    landmarkConstraints.push_back(constraint);
}

void BasicConstraints::addConstraint(const KeyframeConstraint<slam3d::SensorState, Vector3> constraint)
{
    keyframeConstraints.push_back(constraint);
}

void BasicConstraints::visitConstraints(IConstraintVisitor<slam3d::SensorState, Vector3>& visitor)
{
    for(const auto& constraint : landmarkConstraints)
    {
        visitor.visit(constraint);
    }

    for(const auto& constraint : keyframeConstraints)
    {
        visitor.visit(constraint);
    }
}

bool BasicConstraints::removeKeyframe(const Keyframe<slam3d::SensorState>& keyframe)
{
    auto it = std::remove_if(std::begin(keyframeConstraints), std::end(keyframeConstraints),
                             [&keyframe](KeyframeConstraint<slam3d::SensorState, Vector3>& constraint) {
                                 return constraint.firstKeyframe->id == keyframe.id ||
                                        constraint.secondKeyframe->id != keyframe.id;
                             });

    keyframeConstraints.erase(it, std::end(keyframeConstraints));

    return true;
}

bool BasicConstraints::removeLandmark(const Landmark<Vector3>& landmark)
{
    auto it = std::remove_if(std::begin(landmarkConstraints), std::end(landmarkConstraints),
                             [&landmark](LandmarkObservationConstraint<slam3d::SensorState, Vector3>& constraint)
                             { return constraint.landmark->id == landmark.id; });

    landmarkConstraints.erase(it, std::end(landmarkConstraints));

    return true;
}

std::optional<Vector3> reconstructPoint(const Eigen::Vector2i& imgPoint, const float depth,
                                        const Eigen::Vector2f& principalPoint, const Eigen::Vector2f& invFocal)
{
    if(!isDepthValid(depth))
    {
        return std::nullopt;
    }

    const auto z = depth;
    const auto x = (static_cast<float>(imgPoint.x()) - principalPoint.x()) * z * invFocal.x();
    const auto y = (static_cast<float>(imgPoint.y()) - principalPoint.y()) * z * invFocal.y();

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

RgbdFeatureFrontend::RgbdFeatureFrontend(
    std::shared_ptr<IPnpAlgorithm<slam3d::SensorState, Vector3>> newTracker,
    std::shared_ptr<IFeatureDetector<RgbFrame, Eigen::Vector2f, float, 32>> newDetector,
    std::shared_ptr<IFeatureMatcher<Eigen::Vector2f, float, 32>> newMatcher,
    std::shared_ptr<IMapComponentsFactory<slam3d::SensorState, slam3d::LandmarkState>> newMapComponentsFactory)
{
    tracker = std::move(newTracker);
    detector = std::move(newDetector);
    matcher = std::move(newMatcher);
    mapComponentsFactory = std::move(newMapComponentsFactory);
}

void RgbdFeatureFrontend::visitLandmarks(LandmarkVisitor<Vector3>& visitor)
{
    std::vector<std::shared_ptr<rgbd::Landmark>> landmarks;
    for(auto& landmark : landmarks)
    {
        visitor.visit(landmark);
    }
}

void RgbdFeatureFrontend::visitKeyframes(KeyframeVisitor<slam3d::SensorState>& visitor)
{
    for(auto& keyframe : keyframes)
    {
        visitor.visit(keyframe);
    }
}

bool RgbdFeatureFrontend::isNewKeyframeRequired(const std::size_t matchedLandmarks) const
{
    const auto minMatchedLandmarks = static_cast<std::size_t>(
        std::get<float>(parametersHandler->getParameter("rgbd_feature_frontend/new_keyframe_min_landmarks").value()));

    return matchedLandmarks < minMatchedLandmarks;
}

bool RgbdFeatureFrontend::isLoopClosureNeeded() const
{
    return false;
}

std::shared_ptr<rgbd::Keyframe> RgbdFeatureFrontend::addKeyframe(const RgbdFrame& /*sensorData*/,
                                                                 const slam3d::SensorState& pose)
{
    auto newKeyframe = mapComponentsFactory->createKeyframe();
    newKeyframe->state = pose;
    keyframes.push_back(newKeyframe);

    spdlog::debug("Added new keyframe: {}, pos: ({}, {}, {}), quat: [{} {} {} {}]", newKeyframe->id, pose.position.x(),
                  pose.position.y(), pose.position.z(), pose.orientation.x(), pose.orientation.y(),
                  pose.orientation.z(), pose.orientation.w());

    return newKeyframe;
}

std::shared_ptr<RgbdFeatureFrontend::Constraints> RgbdFeatureFrontend::prepareConstraints(const RgbdFrame& sensorData)
{
    const auto keypoints = detector->detect(sensorData.rgb);

    if(!hasInitialKeyframe())
    {
        initFirstKeyframe(sensorData, keypoints);
        return constraints;
    }

    if(const auto trackSuccessful = track(sensorData, keypoints))
    {
        if(isLoopClosureNeeded())
        {
            // TODO: run loop closure  detection
            auto candidateKeyframe = loopDetector->detectLoop();

            closeLoop(candidateKeyframe);
        }

        constraints->setState(currentPose);
        return constraints;
    }

    if(auto newReferenceKeyframe = relocalize(); newReferenceKeyframe != nullptr)
    {
        referenceKeyframe = std::move(newReferenceKeyframe);

        // TODO: update constraints
        return constraints;
    }

    spdlog::error("Relocalization failed. Tracking lost");
    return nullptr;
}

std::vector<KeypointLandmarkMatch<Eigen::Vector2f, Vector3>>
RgbdFeatureFrontend::matchLandmarks(const std::vector<KeypointDescriptor<Eigen::Vector2f, float>>& keypoints)
{
    const auto [refKeyframeKeypoints, landmarks] = getLandmarksWithKeypoints(referenceKeyframe);
    const auto matches = matcher->match(refKeyframeKeypoints, keypoints);
    std::vector<KeypointLandmarkMatch<Eigen::Vector2f, Vector3>> result;

    for(const auto& match : matches)
    {
        KeypointMatch<Eigen::Vector2f> keypointMatch = {refKeyframeKeypoints[match.fromIndex].keypoint,
                                                        keypoints[match.toIndex].keypoint};
        const auto landmark = landmarks[match.fromIndex];
        KeypointLandmarkMatch<Eigen::Vector2f, Vector3> landmarkMatch = {keypointMatch, landmark};

        result.push_back(landmarkMatch);
    }

    return result;
}

std::pair<std::vector<KeypointDescriptor<Eigen::Vector2f, float>>, std::vector<std::shared_ptr<rgbd::Landmark>>>
RgbdFeatureFrontend::getLandmarksWithKeypoints(std::shared_ptr<rgbd::Keyframe> keyframe)
{
    std::vector<std::shared_ptr<rgbd::Landmark>> keyframeLandmarks = keyframesWithLandmarks[keyframe];
    std::vector<KeypointDescriptor<Eigen::Vector2f, float>> keypoints;
    for(const auto& landmark : keyframeLandmarks)
    {
        const auto keypoint = landmarkDescriptors.find(landmark);
        keypoints.push_back(keypoint->second);
    }

    return std::make_pair(keypoints, keyframeLandmarks);
}

std::shared_ptr<RgbdFeatureFrontend::Constraints>
RgbdFeatureFrontend::track(const RgbdFrame& sensorData,
                           const std::vector<KeypointDescriptor<Eigen::Vector2f, float>>& keypoints)
{
    auto& trackConstraints = constraints;
    const auto matchedLandmarks = matchLandmarks(keypoints);
    auto points = pointsFromRgbdKeypoints(sensorData.depth, matchedLandmarks);
    std::vector<Vector2> cameraPointsForTracking;
    std::vector<std::shared_ptr<rgbd::Landmark>> landmarksForTracking;

    for(const auto& [point, matchedLandmark] : boost::combine(points, matchedLandmarks))
    {
        if(point.has_value() && matchedLandmark.landmark != nullptr)
        {
            cameraPointsForTracking.push_back(matchedLandmark.match.matchedKeypoint.coordinates);
            landmarksForTracking.push_back(matchedLandmark.landmark);
        }
    }

    const auto pointsMatchedCount = cameraPointsForTracking.size();
    spdlog::debug("Tracking points with reference keyframe matched {}", pointsMatchedCount);

    if(pointsMatchedCount < minMatchedPoints())
    {
        spdlog::debug("Too few matches for tracking");
        return nullptr;
    }

    tracker->setCameraParameters(sensorData.depth.cameraParameters);
    auto pose = tracker->solvePnp(landmarksForTracking, cameraPointsForTracking, currentPose);

    if(!pose)
    {
        spdlog::debug("Reference frame tracking failed");
        return nullptr;
    }

    currentPose = pose.value();

    if(isBetterReferenceKeyframeNeeded(pointsMatchedCount))
    {
        auto bestReferenceKeyframe = findBetterReferenceKeyframe(pose.value(), sensorData);
        if(bestReferenceKeyframe != nullptr && bestReferenceKeyframe != referenceKeyframe)
        {
            spdlog::debug("Found better ref keyframe: {}", bestReferenceKeyframe->id);
            referenceKeyframe = bestReferenceKeyframe;
        }
    }

    if(isNewKeyframeRequired(pointsMatchedCount))
    {
        auto newKeyframe = addKeyframe(sensorData, pose.value());

        auto landmarksOnFrame = findVisibleLocalLandmarks(pose.value(), sensorData);
        const auto newLandmarkKeypoints = findKeypointsForNewLandmarks(keypoints, landmarksOnFrame);

        addNewLandmarks(newLandmarkKeypoints, newKeyframe, sensorData, trackConstraints);
        updateVisibleLandmarks(landmarksOnFrame, newKeyframe);

        referenceKeyframe = std::move(newKeyframe);
    }

    return trackConstraints;
}

void RgbdFeatureFrontend::updateVisibleLandmarks(const std::vector<std::shared_ptr<rgbd::Landmark>>& landmarksOnFrame,
                                                 const std::shared_ptr<rgbd::Keyframe>& keyframe)
{
    for(const auto& landmark : landmarksOnFrame)
    {
        markLandmarkInKeyframe(landmark, keyframe);
    }
}

void RgbdFeatureFrontend::addNewLandmarks(
    const std::vector<KeypointDescriptor<Eigen::Vector2f, float>>& newLandmarkKeypoints,
    std::shared_ptr<rgbd::Keyframe> newKeyframe, const RgbdFrame& sensorData,
    std::shared_ptr<ConstraintsInterface<slam3d::SensorState, Vector3>> constraints)
{
    const Eigen::Vector2f invFocal = 1.0f / sensorData.depth.cameraParameters.focal.array();

    for(const auto& keypoint : newLandmarkKeypoints)
    {
        const auto keypointCoordinate = keypoint.keypoint.coordinates.cast<int>();
        const auto landmarkCoordinatesInKeyframe =
            reconstructPoint(keypointCoordinate, getDepth(sensorData.depth, keypointCoordinate),
                             sensorData.depth.cameraParameters.principalPoint, invFocal);

        if(landmarkCoordinatesInKeyframe.has_value())
        {
            auto state = toGlobalCoordinates(landmarkCoordinatesInKeyframe.value(), newKeyframe->state);
            auto landmark = addLandmark(newKeyframe, state, keypoint);

            LandmarkObservationConstraint<slam3d::SensorState, Vector3> landmarkConstraint{
                newKeyframe, landmark, landmarkCoordinatesInKeyframe.value()};
            constraints->addConstraint(landmarkConstraint);
        }
    }
}

void RgbdFeatureFrontend::initFirstKeyframe(const RgbdFrame& sensorData,
                                            const std::vector<KeypointDescriptor<Eigen::Vector2f, float>>& keypoints)
{
    constraints = std::make_shared<BasicConstraints>();

    const slam3d::SensorState pose = {Vector3(0, 0, 0), Quaternion::Identity()};
    auto keyframe = addKeyframe(sensorData, pose);
    const Eigen::Vector2f invFocal = 1.0 / sensorData.depth.cameraParameters.focal.array();

    for(const auto& keypointWithDescriptor : keypoints)
    {
        const auto keypointCoordinates = keypointWithDescriptor.keypoint.coordinates.cast<int>();
        const auto landmarkCoordinates =
            reconstructPoint(keypointCoordinates, getDepth(sensorData.depth, keypointCoordinates),
                             sensorData.depth.cameraParameters.principalPoint, invFocal);

        if(landmarkCoordinates.has_value())
        {
            auto landmark = addLandmark(keyframe, landmarkCoordinates.value(), keypointWithDescriptor);

            LandmarkObservationConstraint<slam3d::SensorState, Vector3> landmarkConstraint{keyframe, landmark,
                                                                                           landmarkCoordinates.value()};
            constraints->addConstraint(landmarkConstraint);
        }
    }

    constraints->setState(pose);
    referenceKeyframe = std::move(keyframe);
    currentPose = pose;
}

std::size_t RgbdFeatureFrontend::minMatchedPoints() const
{
    const auto minMatchedLandmarks =
        std::get<float>(parametersHandler->getParameter("rgbd_feature_frontend/min_matched_points").value());

    return static_cast<std::size_t>(minMatchedLandmarks);
}

std::vector<std::shared_ptr<rgbd::Landmark>>
RgbdFeatureFrontend::findVisibleLocalLandmarks(const slam3d::SensorState& pose, const RgbdFrame& sensorData) const
{
    std::vector<std::shared_ptr<rgbd::Landmark>> localLandmarks;

    boost::copy(landmarks |
                    boost::adaptors::filtered(
                        [&cameraParameters = sensorData.depth.cameraParameters, &imgSize = sensorData.depth.size,
                         &pose](const std::shared_ptr<rgbd::Landmark>& landmark)
                        { return isVisibleInFrame(landmark->state, pose, cameraParameters, imgSize); }) |
                    boost::adaptors::uniqued,
                std::back_inserter(localLandmarks));

    return localLandmarks;
}

std::vector<KeypointDescriptor<Eigen::Vector2f, float>> RgbdFeatureFrontend::findKeypointsForNewLandmarks(
    const std::vector<KeypointDescriptor<Eigen::Vector2f, float>>& keypoints,
    const boost::span<std::shared_ptr<rgbd::Landmark>> landmarks) const
{
    std::vector<KeypointDescriptor<Eigen::Vector2f, float>> localLandmarksDescriptors;
    localLandmarksDescriptors.reserve(landmarks.size());

    std::transform(
        std::begin(landmarks), std::end(landmarks), std::back_inserter(localLandmarksDescriptors),
        [&landmarkDescriptors = landmarkDescriptors](const std::shared_ptr<Landmark<Eigen::Vector3f>>& landmark)
        {
            auto it = landmarkDescriptors.find(landmark);
            return it->second;
        });

    auto matches = matcher->match(localLandmarksDescriptors, keypoints);

    spdlog::debug("Looking for keypoints for new landmarks, visible: {}, matched: {}", landmarks.size(),
                  matches.size());

    std::vector<std::size_t> allIndices, matchedIndices, newKeypointsIndices;
    allIndices.resize(keypoints.size());
    std::iota(std::begin(allIndices), std::end(allIndices), 0);

    std::transform(std::begin(matches), std::end(matches), std::back_inserter(matchedIndices),
                   [](const DescriptorMatch& match) { return match.toIndex; });

    std::sort(std::begin(matchedIndices), std::end(matchedIndices));

    std::set_difference(std::begin(allIndices), std::end(allIndices), std::begin(matchedIndices),
                        std::end(matchedIndices), std::back_inserter(newKeypointsIndices));

    std::vector<KeypointDescriptor<Eigen::Vector2f, float>> newKeypoints;
    std::transform(std::begin(newKeypointsIndices), std::end(newKeypointsIndices), std::back_inserter(newKeypoints),
                   [&keypoints](std::size_t index) { return keypoints[index]; });

    return newKeypoints;
}

std::shared_ptr<rgbd::Landmark>
RgbdFeatureFrontend::addLandmark(std::shared_ptr<rgbd::Keyframe>& keyframe, const Vector3& state,
                                 const KeypointDescriptor<Eigen::Vector2f, float>& keypoint)
{
    auto newLandmark = mapComponentsFactory->createLandmark();
    newLandmark->state = state;
    landmarks.push_back(newLandmark);

    bindLandmark(keypoint, newLandmark);
    markLandmarkInKeyframe(newLandmark, keyframe);

    return newLandmark;
}

void RgbdFeatureFrontend::bindLandmark(const KeypointDescriptor<Eigen::Vector2f, float>& keypointWithDescriptor,
                                       std::shared_ptr<rgbd::Landmark>& landmark)
{
    landmarkDescriptors.insert(std::make_pair(landmark, keypointWithDescriptor));
}

void RgbdFeatureFrontend::markLandmarkInKeyframe(const std::shared_ptr<rgbd::Landmark>& landmark,
                                                 const std::shared_ptr<rgbd::Keyframe>& keyframe)
{
    keyframesWithLandmarks[keyframe].push_back(landmark);
    landmarksInKeyframes[landmark].push_back(keyframe);
}

std::shared_ptr<rgbd::Keyframe> RgbdFeatureFrontend::relocalize()
{
    // TODO: implement
    return nullptr;
}

bool RgbdFeatureFrontend::isBetterReferenceKeyframeNeeded(const std::size_t keyframeLandmarksCount) const
{
    const auto maxLandmarks = static_cast<std::size_t>(
        std::get<float>(parametersHandler->getParameter("rgbd_feature_frontend/better_keyframe_landmarks").value()));

    return keyframeLandmarksCount < maxLandmarks;
}

std::shared_ptr<rgbd::Keyframe> RgbdFeatureFrontend::findBetterReferenceKeyframe(const slam3d::SensorState& pose,
                                                                                 const RgbdFrame& frame) const
{
    std::map<std::shared_ptr<rgbd::Keyframe>, int> keyframeCount;
    // TODO: optimize, just get local landmarks
    for(const auto& [landmark, keyframesForLandmark] : landmarksInKeyframes)
    {
        const auto point = toCameraCoordinates(landmark->state, pose);

        if(isVisibleInFrame(point, frame.depth.cameraParameters, frame.rgb.size))
        {
            for(const auto& keyframe : keyframesForLandmark)
            {
                keyframeCount[keyframe] += 1;
            }
        }
    }

    auto foundIt = std::max_element(std::begin(keyframeCount), std::end(keyframeCount),
                                    [](const std::pair<std::shared_ptr<rgbd::Keyframe>, int>& firstPair,
                                       const std::pair<std::shared_ptr<rgbd::Keyframe>, int>& firstSecond)
                                    { return firstPair.second < firstSecond.second; });

    return foundIt != std::end(keyframeCount) ? foundIt->first : nullptr;
}

void RgbdFeatureFrontend::closeLoop(std::shared_ptr<Keyframe<slam3d::SensorState>>& loopCandidate)
{
    // TODO: implement
}

} // namespace mslam
