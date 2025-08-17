#include "modular_slam/frontend/rgbd_feature_frontend.hpp"
#include "modular_slam/depth_frame.hpp"

#include "modular_slam/loop_detection.hpp"
#include "modular_slam/observation.hpp"
#include "modular_slam/orb_feature.hpp"
#include "modular_slam/orb_relocalizer.hpp"
#include "modular_slam/projection.hpp"

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
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mslam
{

struct NeighboursVisitor : IMapVisitor<rgbd::SensorState, rgbd::LandmarkState, rgbd::RgbdOrbKeypointDescriptor>
{
    void visit(std::shared_ptr<rgbd::Landmark> landmark) override { landmarks.insert(landmark); }
    std::unordered_set<std::shared_ptr<rgbd::Landmark>> landmarks;
};

struct ObservationsVisitor : IMapVisitor<rgbd::SensorState, rgbd::LandmarkState, rgbd::RgbdOrbKeypointDescriptor>
{
    void visit(const LandmarkKeyframeObservation<slam3d::SensorState, slam3d::LandmarkState,
                                                 rgbd::RgbdOrbKeypointDescriptor>& observation) override
    {
        landmarks[observation.landmark].insert(observation.keyframe);
    }

    std::unordered_map<std::shared_ptr<rgbd::Landmark>, std::unordered_set<std::shared_ptr<rgbd::Keyframe>>> landmarks;
};

struct RecentObservationsVisitor : IMapVisitor<rgbd::SensorState, rgbd::LandmarkState, rgbd::RgbdOrbKeypointDescriptor>
{
    void visit(const LandmarkKeyframeObservation<slam3d::SensorState, slam3d::LandmarkState,
                                                 rgbd::RgbdOrbKeypointDescriptor>& observation) override
    {
        auto it = landmarksObservations.find(observation.landmark);
        const bool add = it == std::end(landmarksObservations);
        const bool update = !add && observation.keyframe->id > it->second.keyframe->id;

        if(add)
        {
            landmarksObservations[observation.landmark] = observation;
        }
        else if(update)
        {
            it->second = observation;
        }
    }

    std::unordered_map<
        std::shared_ptr<rgbd::Landmark>,
        LandmarkKeyframeObservation<slam3d::SensorState, slam3d::LandmarkState, rgbd::RgbdOrbKeypointDescriptor>>
        landmarksObservations;
};

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
std::vector<std::optional<Vector3>>
pointsFromRgbdKeypoints(const DepthFrame& depthFrame,
                        const std::vector<KeypointDescriptor<std::uint8_t, 32>>& matchedKeypoints)
{
    std::vector<std::optional<Vector3>> points;
    points.reserve(matchedKeypoints.size());

    const Vector2 invFocal = 1.0 / depthFrame.cameraParameters.focal.array();
    for(const auto& keypoint : matchedKeypoints)
    {
        const auto imgPoint = keypoint.keypoint.coordinates.cast<int>();
        const auto depth = getDepth(depthFrame, imgPoint);
        const auto point = reconstructPoint(keypoint.keypoint.coordinates, depth,
                                            depthFrame.cameraParameters.principalPoint, invFocal);

        points.push_back(point);
    }

    return points;
}

RgbdFeatureFrontend::RgbdFeatureFrontend(
    std::shared_ptr<IPnpAlgorithm<slam3d::SensorState, Vector3>> newTracker,
    std::shared_ptr<IFeatureDetector<RgbFrame, std::uint8_t, 32>> newDetector,
    std::shared_ptr<IFeatureMatcher<std::uint8_t, 32>> newMatcher,
    std::shared_ptr<IMapComponentsFactory<slam3d::SensorState, slam3d::LandmarkState>> newMapComponentsFactory,
    std::shared_ptr<IMap<rgbd::SensorState, rgbd::LandmarkState, rgbd::RgbdOrbKeypointDescriptor>> newMap)
{
    pnpAlgorithm = std::move(newTracker);
    detector = std::move(newDetector);
    matcher = std::move(newMatcher);
    mapComponentsFactory = std::move(newMapComponentsFactory);
    map = std::move(newMap);

    relocalizer = std::make_unique<OrbRelocalizer>();
}

bool RgbdFeatureFrontend::isNewKeyframeRequired(const std::size_t numOfInliers) const
{
    const auto minMatchedLandmarks = static_cast<std::size_t>(
        std::get<float>(parametersHandler->getParameter("rgbd_feature_frontend/new_keyframe_min_landmarks").value()));

    return numOfInliers < minMatchedLandmarks;
}

bool RgbdFeatureFrontend::isLoopClosureNeeded() const
{
    return false;
}

std::shared_ptr<rgbd::Keyframe>
RgbdFeatureFrontend::addKeyframe(const RgbdFrame& /*sensorData*/, const slam3d::SensorState& pose,
                                 const std::vector<KeypointDescriptor<std::uint8_t, 32>>& keypoints)
{
    auto newKeyframe = mapComponentsFactory->createKeyframe();
    newKeyframe->state = pose;

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

    const auto isTrackingSuccessful = !output.landmarkObservations.empty();

    if(isTrackingSuccessful)
    {
        if(isLoopClosureNeeded())
        {
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
}

std::pair<std::vector<KeypointDescriptor<std::uint8_t, 32>>, std::vector<std::shared_ptr<rgbd::Landmark>>>
RgbdFeatureFrontend::matchLandmarks(const std::vector<KeypointDescriptor<std::uint8_t, 32>>& keypoints,
                                    std::shared_ptr<rgbd::Keyframe> keyframe)
{
    const auto [keyframeKeypoints, landmarks] = getLandmarksWithKeypoints(keyframe);
    const auto matches = matcher->match(keypoints, keyframeKeypoints);

    std::vector<KeypointLandmarkMatch<Vector3>> result;
    std::vector<KeypointDescriptor<std::uint8_t, 32>> matchedKeypoints;
    std::vector<std::shared_ptr<rgbd::Landmark>> matchedLandmarks;
    // TODO: use boost geometry rtree for keypoints

    for(const auto& match : matches)
    {
        const auto& keypoint = keypoints[match.fromIndex];
        auto landmark = landmarks[match.toIndex];

        matchedKeypoints.push_back(keypoint);
        matchedLandmarks.emplace_back(std::move(landmark));
    }

    return std::make_pair(std::move(matchedKeypoints), std::move(matchedLandmarks));
}

std::pair<std::vector<KeypointDescriptor<std::uint8_t, 32>>, std::vector<std::shared_ptr<rgbd::Landmark>>>
RgbdFeatureFrontend::getLandmarksWithKeypoints(std::shared_ptr<rgbd::Keyframe> keyframe)
{
    std::pair<std::vector<KeypointDescriptor<std::uint8_t, 32>>, std::vector<std::shared_ptr<rgbd::Landmark>>> result;

    RecentObservationsVisitor visitor;
    MapVisitingParams visitingParams;

    visitingParams.observationParams.graphParams = std::make_optional<GraphBasedParams>({keyframe->id, 2});
    map->visit(visitor, visitingParams);

    for(auto& [landmark, observation] : visitor.landmarksObservations)
    {
        const KeypointDescriptor<std::uint8_t, 32> keypointWithDescriptor = {observation.observation.keypoint.keypoint,
                                                                             observation.observation.descriptor};

        result.first.push_back(keypointWithDescriptor);
        result.second.push_back(landmark);
    }

    return result;
}

RgbdFeatureFrontend::FrontendOutputType
RgbdFeatureFrontend::track(const RgbdFrame& sensorData, std::vector<KeypointDescriptor<std::uint8_t, 32>>& keypoints)
{
    FrontendOutputType output;
    output.timestamp = sensorData.timestamp;

    const auto [matchedKeypoints, matchedLandmarks] = matchLandmarks(keypoints, referenceKeyframe);
    const auto points = pointsFromRgbdKeypoints(sensorData.depth, matchedKeypoints);
    std::vector<Vector2> cameraPointsForTracking;
    std::vector<std::shared_ptr<rgbd::Landmark>> landmarksForTracking;
    std::vector<LandmarkObservation<rgbd::LandmarkState, rgbd::RgbdOrbKeypointDescriptor>> landmarksObservations;

    cv::Mat cvImg{sensorData.rgb.size.height, sensorData.rgb.size.width, CV_8UC3,
                  const_cast<std::uint8_t*>(sensorData.rgb.data.data())};

    cv::Mat imgWithKeypoints;
    cvImg.copyTo(imgWithKeypoints);

    for(auto i = 0; i < matchedKeypoints.size(); ++i)
    {
        const auto& keypoint = matchedKeypoints[i].keypoint.coordinates;
        auto landmark = projectOnImage(matchedLandmarks[i]->state, sensorData.depth.cameraParameters, currentPose);
        cv::circle(imgWithKeypoints, cv::Point(keypoint.x(), keypoint.y()), 2, cv::Scalar(255, 0, 0), -1);
        cv::circle(imgWithKeypoints, cv::Point(landmark.x(), landmark.y()), 2, cv::Scalar(0, 0, 255), -1);

        cv::line(imgWithKeypoints, cv::Point(keypoint.x(), keypoint.y()), cv::Point(landmark.x(), landmark.y()),
                 cv::Scalar(0, 255, 0));
    }

    cv::imshow("imgWithKpts", imgWithKeypoints);
    cv::pollKey();

    cameraPointsForTracking.reserve(points.size());
    landmarksForTracking.reserve(points.size());

    std::vector<std::size_t> usedKeypointIndices;

    std::size_t index = 0;
    for(const auto& [point, matchedLandmark, keypoint] : boost::combine(points, matchedLandmarks, matchedKeypoints))
    {
        if(point.has_value())
        {
            cameraPointsForTracking.push_back(keypoint.keypoint.coordinates);
            landmarksForTracking.push_back(matchedLandmark);
            rgbd::RgbdKeypointDescriptor<std::uint8_t, 32> rgbdKeypointDescriptor;
            rgbdKeypointDescriptor.descriptor = keypoint.descriptor;
            rgbdKeypointDescriptor.keypoint.keypoint = keypoint.keypoint;
            rgbdKeypointDescriptor.keypoint.depth = point->z();
            landmarksObservations.push_back({matchedLandmark, rgbdKeypointDescriptor});

            // usedKeypointIndices.insert(index);
            usedKeypointIndices.push_back(index);
        }

        ++index;
    }

    const auto pointsMatchedCount = cameraPointsForTracking.size();
    spdlog::debug("Tracking points with reference keyframe {} matched {}", referenceKeyframe->id, pointsMatchedCount);

    if(pointsMatchedCount < minMatchedPoints())
    {
        spdlog::debug("Too few matches for tracking");
        return output;
    }

    pnpAlgorithm->setCameraParameters(sensorData.depth.cameraParameters);

    const auto pnpResult = pnpAlgorithm->solvePnp(landmarksForTracking, cameraPointsForTracking, currentPose);

    if(!pnpResult)
    {
        spdlog::debug("Reference frame tracking failed");
        return output;
    }

    const auto numOfInliers = pnpResult->inliers.count();

    currentPose = pnpResult->pose;
    output.pose = currentPose;

    boost::range::copy(landmarksObservations | boost::adaptors::indexed(0) |
                           boost::adaptors::filtered([&inliers = pnpResult->inliers](auto&& range)
                                                     { return inliers[static_cast<std::size_t>(range.index())]; }) |
                           boost::adaptors::transformed([](auto&& range) { return range.value(); }),
                       std::back_inserter(output.landmarkObservations));

    auto bestReferenceKeyframe = findBetterReferenceKeyframe(output, sensorData);
    if(bestReferenceKeyframe != nullptr && bestReferenceKeyframe != referenceKeyframe)
    {
        spdlog::debug("Found better ref keyframe: {}", bestReferenceKeyframe->id);
        referenceKeyframe = bestReferenceKeyframe;
    }

    if(isNewKeyframeRequired(numOfInliers))
    {
        auto newKeyframe = addKeyframe(sensorData, currentPose, keypoints);

        std::set<std::size_t> allKeypointIndices;
        for(int i = 0; i < keypoints.size(); i++)
            allKeypointIndices.insert(i);

        std::set<std::size_t> newKeypointIndices;

        std::set_difference(std::begin(allKeypointIndices), std::end(allKeypointIndices),
                            std::begin(usedKeypointIndices), std::end(usedKeypointIndices),
                            std::inserter(newKeypointIndices, std::begin(newKeypointIndices)));

        std::vector<KeypointDescriptor<std::uint8_t, 32>> newLandmarkKeypoints;

        std::transform(std::begin(newKeypointIndices), std::end(newKeypointIndices),
                       std::back_inserter(newLandmarkKeypoints),
                       [&keypoints](std::size_t index) { return keypoints[index]; });

        addNewLandmarks(newLandmarkKeypoints, newKeyframe, sensorData, output);

        output.newKeyframe = newKeyframe;
        referenceKeyframe = std::move(newKeyframe);
    }

    return output;
}

void RgbdFeatureFrontend::addNewLandmarks(const std::vector<KeypointDescriptor<std::uint8_t, 32>>& newLandmarkKeypoints,
                                          std::shared_ptr<rgbd::Keyframe> newKeyframe, const RgbdFrame& sensorData,
                                          FrontendOutputType& output)
{
    const Vector2 invFocal = 1.0 / sensorData.depth.cameraParameters.focal.array();
    constexpr auto zThreshold = 3.f;

    for(const auto& keypoint : newLandmarkKeypoints)
    {
        const auto keypointCoordinate = keypoint.keypoint.coordinates.cast<int>();
        const auto landmarkCoordinatesInKeyframe =
            reconstructPoint(keypoint.keypoint.coordinates, getDepth(sensorData.depth, keypointCoordinate),
                             sensorData.depth.cameraParameters.principalPoint, invFocal);

        if(landmarkCoordinatesInKeyframe.has_value() && landmarkCoordinatesInKeyframe->z() <= zThreshold)
        {
            auto state = toGlobalCoordinates(landmarkCoordinatesInKeyframe.value(), newKeyframe->state);
            auto landmark = addLandmark(newKeyframe, state, keypoint);

            rgbd::LandmarkObservation landmarkObservation{
                landmark, {{keypoint.keypoint, landmarkCoordinatesInKeyframe->z()}, keypoint.descriptor}};

            output.landmarkObservations.push_back(landmarkObservation);
            output.newLandmarks.push_back(landmark);
        }
    }

    spdlog::info("Added {} new landmarks, old {}", output.newLandmarks.size(),
                 output.landmarkObservations.size() - output.newLandmarks.size());
}

RgbdFeatureFrontend::FrontendOutputType
RgbdFeatureFrontend::initFirstKeyframe(const RgbdFrame& sensorData,
                                       const std::vector<KeypointDescriptor<std::uint8_t, 32>>& keypoints)
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

std::shared_ptr<rgbd::Landmark> RgbdFeatureFrontend::addLandmark(std::shared_ptr<rgbd::Keyframe>& keyframe,
                                                                 const Vector3& state,
                                                                 const KeypointDescriptor<std::uint8_t, 32>& keypoint)
{
    auto newLandmark = mapComponentsFactory->createLandmark();
    newLandmark->state = state;

    return newLandmark;
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

std::shared_ptr<rgbd::Keyframe>
RgbdFeatureFrontend::relocalize(std::vector<KeypointDescriptor<std::uint8_t, 32>>& keypoints)
{
    return nullptr;
    // std::vector<std::shared_ptr<rgbd::Keyframe>> keyframeCandidates = relocalizer->relocalize(keypoints);

    // if(keyframeCandidates.empty())
    //     return nullptr;

    // auto relocalizedCandidates =
    //     keyframeCandidates |
    //     boost::adaptors::transformed(
    //         [this, &keypoints](std::shared_ptr<rgbd::Keyframe> keyframe) -> RelocalizationResult
    //         {
    //             const auto matches = matchLandmarks(keypoints, keyframe);
    //             std::vector<Vector2> cameraPointsForTracking;
    //             std::vector<std::shared_ptr<rgbd::Landmark>> landmarksForTracking;

    //             for(const auto& landmarkMatch : matches)
    //             {
    //                 cameraPointsForTracking.push_back(landmarkMatch.match.matchedKeypoint.coordinates);
    //                 landmarksForTracking.push_back(landmarkMatch.landmark);
    //             }

    //             auto result = pnpAlgorithm->solvePnp(landmarksForTracking, cameraPointsForTracking);

    //             if(!result.has_value())
    //                 return {nullptr, 0.0};

    //             return {keyframe, static_cast<double>(result->inliers.count())};
    //         });

    // auto result = boost::range::max_element(
    //     relocalizedCandidates, [](const RelocalizationResult& firstREsult, const RelocalizationResult& secondResult)
    //     { return firstREsult.score < secondResult.score; });

    // constexpr auto scoreThreshold = 60;

    // return result->score >= scoreThreshold ? result->keyframe : result->keyframe;
}

bool RgbdFeatureFrontend::isBetterReferenceKeyframeNeeded(const std::size_t keyframeLandmarksCount) const
{
    const auto maxLandmarks = static_cast<std::size_t>(
        std::get<float>(parametersHandler->getParameter("rgbd_feature_frontend/better_keyframe_landmarks").value()));

    return keyframeLandmarksCount < maxLandmarks;
}

std::shared_ptr<rgbd::Keyframe> RgbdFeatureFrontend::findBetterReferenceKeyframe(FrontendOutputType& result,
                                                                                 const RgbdFrame& frame) const
{
    std::map<std::shared_ptr<rgbd::Keyframe>, int> keyframeCount;

    ObservationsVisitor visitor;
    MapVisitingParams visitingParams;
    visitingParams.observationParams.graphParams = std::make_optional<GraphBasedParams>({referenceKeyframe->id, 5});

    map->visit(visitor, visitingParams);

    for(const auto& [landmark, keyframesForLandmark] : visitor.landmarks)
    {
        if(isVisibleInFrame(landmark->state, result.pose, frame.depth.cameraParameters, frame.rgb.size))
        {
            for(const auto& keyframe : keyframesForLandmark)
            {
                keyframeCount[keyframe] += 1;
            }
        }
    }

    for(auto [keyframe, count] : keyframeCount)
        spdlog::info("{}: {}", keyframe->id, count);

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
