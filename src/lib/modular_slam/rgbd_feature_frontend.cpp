#include "modular_slam/frontend/rgbd_feature_frontend.hpp"
#include "modular_slam/basic_types.hpp"
#include "modular_slam/camera_parameters.hpp"
#include "modular_slam/depth_frame.hpp"
#include "modular_slam/feature_interface.hpp"
#include "modular_slam/loop_detection.hpp"
#include "modular_slam/map_interface.hpp"
#include "modular_slam/observation.hpp"
#include "modular_slam/orb_feature.hpp"
#include "modular_slam/orb_relocalizer.hpp"
#include "modular_slam/parameters_handler.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/rgbd_slam_types.hpp"
#include "modular_slam/slam3d_types.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/adaptor/uniqued.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/algorithm/max_element.hpp>
#include <boost/range/combine.hpp>

#include <algorithm>
#include <boost/range/iterator_range_core.hpp>
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

struct NeighboursVisitor : IMapVisitor<rgbd::SensorState, rgbd::LandmarkState, rgbd::RgbdKeypoint>
{
    void visit(std::shared_ptr<rgbd::Landmark> landmark) override { landmarks.insert(landmark); }
    std::unordered_set<std::shared_ptr<rgbd::Landmark>> landmarks;
};

struct ObservationsVisitor : IMapVisitor<rgbd::SensorState, rgbd::LandmarkState, rgbd::RgbdKeypoint>
{
    void visit(const LandmarkKeyframeObservation<slam3d::SensorState, slam3d::LandmarkState, rgbd::RgbdKeypoint>&
                   observation) override
    {
        landmarks[observation.landmark].insert(observation.keyframe);
    }

    std::unordered_map<std::shared_ptr<rgbd::Landmark>, std::unordered_set<std::shared_ptr<rgbd::Keyframe>>> landmarks;
};

Vector2 projectOnImage(const Vector3& point, const CameraParameters& cameraParams)
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

Vector2 projectOnImage(const Vector3& point, const CameraParameters& cameraParams,
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

    return inWidth && inHeight && point.z() > 0.0f;
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

std::optional<Vector3> reconstructPoint(const Vector2& imgPoint, const float depth, const Vector2& principalPoint,
                                        const Vector2& invFocal)
{
    if(!isDepthValid(depth))
    {
        return std::nullopt;
    }

    const auto z = depth;
    const auto x = (imgPoint.x() - principalPoint.x()) * z * invFocal.x();
    const auto y = (imgPoint.y() - principalPoint.y()) * z * invFocal.y();

    return Vector3{x, y, z};
}

/*!
 * \brief Calculates 3D points in camera coordinate system based on points in image coordinate system
 */
std::vector<std::optional<Vector3>> pointsFromRgbdKeypoints(const DepthFrame& depthFrame,
                                                            const std::vector<KeypointLandmarkMatch<Vector3>>& matches)
{
    std::vector<std::optional<Vector3>> points;
    points.reserve(matches.size());

    const Vector2 invFocal = 1.0 / depthFrame.cameraParameters.focal.array();

    for(const auto& match : matches)
    {
        const Eigen::Vector2i imgPoint = match.match.matchedKeypoint.coordinates.cast<int>();
        const auto depth = getDepth(depthFrame, imgPoint);
        const auto point = reconstructPoint(match.match.matchedKeypoint.coordinates, depth,
                                            depthFrame.cameraParameters.principalPoint, invFocal);

        points.push_back(point);
    }

    return points;
}

RgbdFeatureFrontend::RgbdFeatureFrontend(
    std::shared_ptr<IPnpAlgorithm<slam3d::SensorState, Vector3>> newTracker,
    std::shared_ptr<IFeatureDetector<RgbFrame, float, 32>> newDetector,
    std::shared_ptr<IFeatureMatcher<float, 32>> newMatcher,
    std::shared_ptr<IMapComponentsFactory<slam3d::SensorState, slam3d::LandmarkState>> newMapComponentsFactory,
    std::shared_ptr<IMap<rgbd::SensorState, rgbd::LandmarkState, rgbd::RgbdKeypoint>> newMap)
{
    pnpAlgorithm = std::move(newTracker);
    detector = std::move(newDetector);
    matcher = std::move(newMatcher);
    mapComponentsFactory = std::move(newMapComponentsFactory);
    map = std::move(newMap);

    relocalizer = std::make_unique<OrbRelocalizer>();
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

std::shared_ptr<rgbd::Keyframe>
RgbdFeatureFrontend::addKeyframe(const RgbdFrame& /*sensorData*/, const slam3d::SensorState& pose,
                                 const std::vector<KeypointDescriptor<float, 32>>& keypoints)
{
    auto newKeyframe = mapComponentsFactory->createKeyframe();
    newKeyframe->state = pose;
    // keyframes.push_back(newKeyframe);

    relocalizer->addKeyframe(newKeyframe, keypoints);

    spdlog::debug("Added new keyframe: {}, pos: ({}, {}, {}), quat: [{} {} {} {}]", newKeyframe->id, pose.position.x(),
                  pose.position.y(), pose.position.z(), pose.orientation.x(), pose.orientation.y(),
                  pose.orientation.z(), pose.orientation.w());

    return newKeyframe;
}

RgbdFeatureFrontend::FrontendOutputType RgbdFeatureFrontend::processSensorData(std::shared_ptr<RgbdFrame> sensorData)
{
    auto keypoints = detector->detect(sensorData->rgb);

    if(!hasInitialKeyframe())
    {
        return initFirstKeyframe(*sensorData, keypoints);
    }

    FrontendOutputType output = track(*sensorData, keypoints);

    const auto isTrackingSuccessful = output.landmarkObservations.size() > 0;

    if(isTrackingSuccessful)
    {
        if(isLoopClosureNeeded())
        {
            // TODO: run loop closure  detection
            auto candidateKeyframe = loopDetector->detectLoop();

            closeLoop(candidateKeyframe);
        }

        return output;
    }

    spdlog::error("Tracking failed. Performing relocalization");

    if(auto newReferenceKeyframe = relocalize(keypoints); newReferenceKeyframe != nullptr)
    {
        referenceKeyframe = std::move(newReferenceKeyframe);

        return output;
    }

    spdlog::error("Relocalization failed. Tracking lost");

    return output;
}

void RgbdFeatureFrontend::update(const BackendOutputType& backendOutput)
{
    const auto& outliers = backendOutput.outlierObservations;

    std::for_each(std::begin(outliers), std::end(outliers),
                  [this](const rgbd::KeyframeLandmarkObservation& observation) { removeObservation(observation); });

    // TODO: implement
}

std::vector<KeypointLandmarkMatch<Vector3>>
RgbdFeatureFrontend::matchLandmarks(const std::vector<KeypointDescriptor<float, 32>>& keypoints,
                                    std::shared_ptr<rgbd::Keyframe> keyframe)
{
    spdlog::debug("Matching landmarks...");

    const auto [keyframeKeypoints, landmarks] = getLandmarksWithKeypoints(keyframe);
    const auto matches = matcher->match(keyframeKeypoints, keypoints);

    std::vector<KeypointLandmarkMatch<Vector3>> result;

    for(const auto& match : matches)
    {
        KeypointMatch keypointMatch = {keyframeKeypoints[match.fromIndex].keypoint, keypoints[match.toIndex].keypoint};
        const auto landmark = landmarks[match.fromIndex];
        KeypointLandmarkMatch<Vector3> landmarkMatch = {keypointMatch, landmark};
        result.push_back(landmarkMatch);
    }

    return result;
}

std::pair<std::vector<KeypointDescriptor<float, 32>>, std::vector<std::shared_ptr<rgbd::Landmark>>>
RgbdFeatureFrontend::getLandmarksWithKeypoints(std::shared_ptr<rgbd::Keyframe> keyframe)
{
    // auto& indexedByKeyframe = allObservations.get<0>();
    // auto [foundIterator, endIterator] = indexedByKeyframe.equal_range(keyframe);
    // // TODO: get local landmarks and match them

    std::pair<std::vector<KeypointDescriptor<float, 32>>, std::vector<std::shared_ptr<rgbd::Landmark>>> result;

    // std::for_each(foundIterator, endIterator,
    //               [&result](const rgbd::Observation& observation)
    //               {
    //                   KeypointDescriptor<float, 32> keypoint = {observation.keypoint.keypoint.keypoint,
    //                                                             observation.keypoint.descriptor};
    //                   result.first.push_back(keypoint);
    //                   result.second.push_back(observation.landmark);
    //               });

    auto& indexedByLandmark = allObservations.get<1>();

    NeighboursVisitor visitor;
    MapVisitingParams visitingParams;
    visitingParams.landmarkParams.graphParams = std::make_optional<GraphBasedParams>({keyframe->id, 2});
    map->visit(visitor, visitingParams);

    std::for_each(
        std::begin(visitor.landmarks), std::end(visitor.landmarks),
        [&result, &indexedByLandmark](std::shared_ptr<rgbd::Landmark> landmark)
        {
            auto [foundIterator, endIterator] = indexedByLandmark.equal_range(landmark);
            const auto& keypoint = foundIterator->keypoint;

            KeypointDescriptor<float, 32> keypointWithDescriptor = {keypoint.keypoint.keypoint, keypoint.descriptor};
            result.first.push_back(keypointWithDescriptor);
            result.second.push_back(std::move(landmark));
        });

    // auto [foundIterator, endIterator] = indexedByLandmark.equal_range(keyframe);

    spdlog::info("{} {}", visitor.landmarks.size(), result.first.size());

    return result;
}

RgbdFeatureFrontend::FrontendOutputType
RgbdFeatureFrontend::track(const RgbdFrame& sensorData, std::vector<KeypointDescriptor<float, 32>>& keypoints)
{
    RgbdFeatureFrontend::FrontendOutputType output;
    output.timestamp = sensorData.timestamp;

    const auto matchedLandmarks = matchLandmarks(keypoints, referenceKeyframe);
    auto points = pointsFromRgbdKeypoints(sensorData.depth, matchedLandmarks);
    std::vector<Vector2> cameraPointsForTracking;
    std::vector<std::shared_ptr<rgbd::Landmark>> landmarksForTracking;
    std::vector<LandmarkObservation<rgbd::LandmarkState, rgbd::RgbdKeypoint>> landmarksObservations;

    for(const auto& [point, matchedLandmark] : boost::combine(points, matchedLandmarks))
    {
        if(point.has_value())
        {
            cameraPointsForTracking.push_back(matchedLandmark.match.matchedKeypoint.coordinates);
            landmarksForTracking.push_back(matchedLandmark.landmark);
            landmarksObservations.push_back(
                {matchedLandmark.landmark, {matchedLandmark.match.matchedKeypoint, point->z()}});
        }
    }

    const auto pointsMatchedCount = cameraPointsForTracking.size();
    spdlog::debug("Tracking points with reference keyframe matched {}", pointsMatchedCount);

    if(pointsMatchedCount < minMatchedPoints())
    {
        spdlog::debug("Too few matches for tracking");
        return output;
    }

    pnpAlgorithm->setCameraParameters(sensorData.depth.cameraParameters);

    auto pnpResult = pnpAlgorithm->solvePnp(landmarksForTracking, cameraPointsForTracking, currentPose);

    if(!pnpResult)
    {
        spdlog::debug("Reference frame tracking failed");
        return output;
    }

    currentPose = pnpResult->pose;
    output.pose = currentPose;

    boost::range::copy(landmarksObservations | boost::adaptors::indexed(0) |
                           boost::adaptors::filtered([&inliers = pnpResult->inliers](auto&& range)
                                                     { return inliers[static_cast<std::size_t>(range.index())]; }) |
                           boost::adaptors::transformed([](auto&& range) { return range.value(); }),
                       std::back_inserter(output.landmarkObservations));

    if(isBetterReferenceKeyframeNeeded(pointsMatchedCount))
    {
        auto bestReferenceKeyframe = findBetterReferenceKeyframe(currentPose, sensorData);
        if(bestReferenceKeyframe != nullptr && bestReferenceKeyframe != referenceKeyframe)
        {
            spdlog::debug("Found better ref keyframe: {}", bestReferenceKeyframe->id);
            referenceKeyframe = bestReferenceKeyframe;
        }
    }

    if(isNewKeyframeRequired(pointsMatchedCount))
    {
        auto newKeyframe = addKeyframe(sensorData, currentPose, keypoints);
        auto landmarksOnFrame = findVisibleLocalLandmarks(currentPose, sensorData);
        const auto [newLandmarkKeypoints, matchedLandmarks] = findKeypointsForLandmarks(keypoints, landmarksOnFrame);

        addNewLandmarks(newLandmarkKeypoints, newKeyframe, sensorData, output);
        updateVisibleLandmarks(matchedLandmarks, newKeyframe, sensorData);

        output.newKeyframe = newKeyframe;
        referenceKeyframe = std::move(newKeyframe);
    }

    return output;
}

void RgbdFeatureFrontend::updateVisibleLandmarks(
    const std::unordered_map<std::shared_ptr<rgbd::Landmark>, KeypointDescriptor<float, 32>>& matchedLandmarks,
    const std::shared_ptr<rgbd::Keyframe>& keyframe, const RgbdFrame& sensorData)

{
    const Vector2 invFocal = 1.0 / sensorData.depth.cameraParameters.focal.array();
    for(const auto& [landmark, keypoint] : matchedLandmarks)
    {
        const auto landmarkCoordinatesInKeyframe = toCameraCoordinates(landmark->state, currentPose);

        rgbd::LandmarkObservation landmarkObservation{landmark, {keypoint.keypoint, landmarkCoordinatesInKeyframe.z()}};
        addLandmarkObservation(keyframe, landmarkObservation);

        const auto keypointCoordinate = keypoint.keypoint.coordinates.cast<int>();
        const auto coordinatesInFrame =
            reconstructPoint(keypoint.keypoint.coordinates, getDepth(sensorData.depth, keypointCoordinate),
                             sensorData.depth.cameraParameters.principalPoint, invFocal);

        if(coordinatesInFrame.has_value())
            bindKeypointToLandmark(keypoint, coordinatesInFrame.value(), landmark, keyframe);
    }
}

// TODO: remove
void RgbdFeatureFrontend::addLandmarkObservation(std::shared_ptr<rgbd::Keyframe> keyframe,
                                                 const rgbd::LandmarkObservation& observation)
{
    // observations[keyframe].push_back(observation);
}

void RgbdFeatureFrontend::addNewLandmarks(const std::vector<KeypointDescriptor<float, 32>>& newLandmarkKeypoints,
                                          std::shared_ptr<rgbd::Keyframe> newKeyframe, const RgbdFrame& sensorData,
                                          FrontendOutputType& output)
{
    const Vector2 invFocal = 1.0 / sensorData.depth.cameraParameters.focal.array();

    for(const auto& keypoint : newLandmarkKeypoints)
    {
        const auto keypointCoordinate = keypoint.keypoint.coordinates.cast<int>();
        const auto landmarkCoordinatesInKeyframe =
            reconstructPoint(keypoint.keypoint.coordinates, getDepth(sensorData.depth, keypointCoordinate),
                             sensorData.depth.cameraParameters.principalPoint, invFocal);

        constexpr auto zThreshold = 5.f;
        if(landmarkCoordinatesInKeyframe.has_value() && landmarkCoordinatesInKeyframe->z() <= zThreshold)
        {
            auto state = toGlobalCoordinates(landmarkCoordinatesInKeyframe.value(), newKeyframe->state);
            auto landmark = addLandmark(newKeyframe, state, keypoint);

            rgbd::LandmarkObservation landmarkObservation{landmark,
                                                          {keypoint.keypoint, landmarkCoordinatesInKeyframe->z()}};
            addLandmarkObservation(newKeyframe, landmarkObservation);
            output.landmarkObservations.push_back(landmarkObservation);
            output.newLandmarks.push_back(landmark);
        }
    }
}

RgbdFeatureFrontend::FrontendOutputType
RgbdFeatureFrontend::initFirstKeyframe(const RgbdFrame& sensorData,
                                       const std::vector<KeypointDescriptor<float, 32>>& keypoints)
{
    auto output = FrontendOutputType();
    const slam3d::SensorState pose = {Vector3(0, 0, 0), Quaternion::Identity()};
    auto keyframe = addKeyframe(sensorData, pose, keypoints);

    addNewLandmarks(keypoints, keyframe, sensorData, output);

    currentPose = pose;
    output.pose = currentPose;
    output.newKeyframe = keyframe;
    referenceKeyframe = std::move(keyframe);

    return output;
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

    NeighboursVisitor visitor;
    MapVisitingParams visitingParams;
    visitingParams.landmarkParams.graphParams = std::make_optional<GraphBasedParams>({referenceKeyframe->id, 4});
    map->visit(visitor, visitingParams);

    boost::copy(visitor.landmarks |
                    boost::adaptors::filtered(
                        [&cameraParameters = sensorData.depth.cameraParameters, &imgSize = sensorData.depth.size,
                         &pose](const std::shared_ptr<rgbd::Landmark>& landmark)
                        {
                            const auto isVisible = isVisibleInFrame(landmark->state, pose, cameraParameters, imgSize);

                            return isVisible;
                        }),
                std::back_inserter(localLandmarks));

    return localLandmarks;
}

RgbdFeatureFrontend::LandmarkMatches
RgbdFeatureFrontend::findKeypointsForLandmarks(const std::vector<KeypointDescriptor<float>>& keypoints,
                                               const boost::span<std::shared_ptr<rgbd::Landmark>> landmarks) const
{
    std::vector<KeypointDescriptor<float, 32>> localLandmarksDescriptors;
    localLandmarksDescriptors.reserve(landmarks.size());

    std::transform(std::begin(landmarks), std::end(landmarks), std::back_inserter(localLandmarksDescriptors),
                   [&landmarksIndexed = allObservations.get<1>()](const std::shared_ptr<rgbd::Landmark>& landmark)
                   {
                       const auto [foundIt, endIt] = landmarksIndexed.equal_range(landmark);
                       KeypointDescriptor<float, 32> result;
                       result.descriptor = foundIt->keypoint.descriptor;
                       result.keypoint = foundIt->keypoint.keypoint.keypoint;

                       // TODO: can calculate median?
                       return result;
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

    std::vector<KeypointDescriptor<float, 32>> newKeypoints;

    std::transform(std::begin(newKeypointsIndices), std::end(newKeypointsIndices), std::back_inserter(newKeypoints),
                   [&keypoints](std::size_t index) { return keypoints[index]; });

    std::unordered_map<std::shared_ptr<rgbd::Landmark>, KeypointDescriptor<float, 32>> matchedWithLandmarks;
    std::transform(std::begin(matches), std::end(matches),
                   std::inserter(matchedWithLandmarks, matchedWithLandmarks.begin()),
                   [&keypoints, &landmarks](const DescriptorMatch& match)
                   {
                       auto landmark = landmarks[match.fromIndex];

                       return std::make_pair(landmark, keypoints[match.toIndex]);
                   });

    return {newKeypoints, matchedWithLandmarks};
}

std::shared_ptr<rgbd::Landmark> RgbdFeatureFrontend::addLandmark(std::shared_ptr<rgbd::Keyframe>& keyframe,
                                                                 const Vector3& state,
                                                                 const KeypointDescriptor<float, 32>& keypoint)
{
    auto newLandmark = mapComponentsFactory->createLandmark();
    newLandmark->state = state;

    bindKeypointToLandmark(keypoint, state, newLandmark, keyframe);

    return newLandmark;
}

void RgbdFeatureFrontend::bindKeypointToLandmark(const KeypointDescriptor<float, 32>& keypointWithDescriptor,
                                                 const Vector3& coordinatesInCameraFrame,
                                                 std::shared_ptr<rgbd::Landmark> landmark,
                                                 std::shared_ptr<rgbd::Keyframe> keyframe)
{
    // landmarkDescriptors.insert(std::make_pair(landmark, keypointWithDescriptor));

    // keyframesWithLandmarks[keyframe].insert(landmark);
    // landmarksInKeyframes[landmark].insert(keyframe);

    rgbd::Observation observation; // = {keypointWithDescriptor, std::move(landmark), std::move(keyframe)};

    observation.keyframe = keyframe;
    observation.landmark = landmark;
    observation.keypoint.descriptor = keypointWithDescriptor.descriptor;
    observation.keypoint.keypoint.keypoint = keypointWithDescriptor.keypoint;
    observation.keypoint.keypoint.depth = coordinatesInCameraFrame.z();

    allObservations.insert(observation);
}

void RgbdFeatureFrontend::removeObservation(const rgbd::KeyframeLandmarkObservation& observation)
{
    // auto keyframe = observation.keyframe;
    // auto& keyframeLandmarks = keyframesWithLandmarks[keyframe];

    // if(keyframeLandmarks.empty())
    // {
    //     keyframesWithLandmarks.erase(keyframe);
    // }

    // auto& landmarkKeyframes = landmarksInKeyframes[observation.landmark];

    // landmarkKeyframes.erase(observation.keyframe);

    // if(landmarkKeyframes.empty())
    // {
    //     landmarksInKeyframes.erase(observation.landmark);
    // }
}

struct RelocalizationResult
{
    std::shared_ptr<rgbd::Keyframe> keyframe;
    double score;
};

std::shared_ptr<rgbd::Keyframe> RgbdFeatureFrontend::relocalize(std::vector<KeypointDescriptor<float, 32>>& keypoints)
{
    std::vector<std::shared_ptr<rgbd::Keyframe>> keyframeCandidates = relocalizer->relocalize(keypoints);

    if(keyframeCandidates.empty())
        return nullptr;

    auto relocalizedCandidates =
        keyframeCandidates |
        boost::adaptors::transformed(
            [this, &keypoints](std::shared_ptr<rgbd::Keyframe> keyframe) -> RelocalizationResult
            {
                const auto matches = matchLandmarks(keypoints, keyframe);
                std::vector<Vector2> cameraPointsForTracking;
                std::vector<std::shared_ptr<rgbd::Landmark>> landmarksForTracking;

                for(const auto& landmarkMatch : matches)
                {
                    cameraPointsForTracking.push_back(landmarkMatch.match.matchedKeypoint.coordinates);
                    landmarksForTracking.push_back(landmarkMatch.landmark);
                }

                auto result = pnpAlgorithm->solvePnp(landmarksForTracking, cameraPointsForTracking);

                if(!result.has_value())
                    return {nullptr, 0.0};

                return {keyframe, static_cast<double>(result->inliers.count())};
            });

    auto result = boost::range::max_element(
        relocalizedCandidates, [](const RelocalizationResult& firstREsult, const RelocalizationResult& secondResult)
        { return firstREsult.score < secondResult.score; });

    constexpr auto scoreThreshold = 60;

    return result->score >= scoreThreshold ? result->keyframe : result->keyframe;
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

    ObservationsVisitor visitor;
    MapVisitingParams visitingParams;
    visitingParams.landmarkKeyframeObservationParams.graphParams =
        std::make_optional<GraphBasedParams>({referenceKeyframe->id, 5});
    map->visit(visitor, visitingParams);

    for(const auto& [landmark, keyframesForLandmark] : visitor.landmarks)
    {
        if(isVisibleInFrame(landmark->state, pose, frame.depth.cameraParameters, frame.rgb.size))
        {
            for(const auto& keyframe : keyframesForLandmark)
            {
                keyframeCount[keyframe] += 1;
            }
        }
    }

    for(const auto& [keyframe, count] : keyframeCount)
        spdlog::info("Observed by kfr {}: {}", keyframe->id, count);

    const auto foundIt = std::max_element(std::begin(keyframeCount), std::end(keyframeCount),
                                          [](const std::pair<std::shared_ptr<rgbd::Keyframe>, int>& firstPair,
                                             const std::pair<std::shared_ptr<rgbd::Keyframe>, int>& secondPair)
                                          { return firstPair.second < secondPair.second; });

    return foundIt != std::end(keyframeCount) ? foundIt->first : nullptr;
}

void RgbdFeatureFrontend::closeLoop(std::shared_ptr<Keyframe<slam3d::SensorState>>& loopCandidate)
{
    // TODO: implement
}

} // namespace mslam
