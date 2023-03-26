#include "modular_slam/basic_map.hpp"
#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/map_interface.hpp"
#include "modular_slam/rgbd_slam_types.hpp"
#include "modular_slam/slam3d_types.hpp"

#include <algorithm>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <iterator>
#include <queue>

namespace mslam
{

BasicMap::BasicMap() {}

void BasicMap::update(
    const FrontendOutput<slam3d::SensorState, slam3d::LandmarkState, rgbd::RgbdKeypoint>& frontendOutput)
{
    if(frontendOutput.newKeyframe != nullptr)
    {
        addKeyframe(frontendOutput.newKeyframe);

        std::for_each(std::begin(frontendOutput.newLandmarks), std::end(frontendOutput.newLandmarks),
                      [this](auto& landmark) { addLandmark(landmark); });

        std::for_each(std::begin(frontendOutput.landmarkObservations), std::end(frontendOutput.landmarkObservations),
                      [this, keyframe = frontendOutput.newKeyframe](
                          const LandmarkObservation<slam3d::LandmarkState, rgbd::RgbdKeypoint>& observation)
                      {
                          LandmarkKeyframeObservation<slam3d::SensorState, slam3d::LandmarkState, rgbd::RgbdKeypoint>
                              newObservation = {keyframe, observation.landmark, observation.observation};

                          addObservation(newObservation);
                      });

        updateCovisibility(frontendOutput.newKeyframe, frontendOutput.landmarkObservations);
    }
}

template <typename T>
void visitElements(IMapVisitor<slam3d::SensorState, slam3d::LandmarkState, rgbd::RgbdKeypoint>& visitor, T&& container)
{
    std::for_each(std::cbegin(container), std::cend(container),
                  [&visitor](const auto& element) { visitor.visit(element); });
}

void BasicMap::visit(IMapVisitor<slam3d::SensorState, slam3d::LandmarkState, rgbd::RgbdKeypoint>& visitor,
                     const MapVisitingParams& params)
{
    if(isVisitKeyframeFlagSet(params))
    {
        visitElements(visitor, keyframes);
    }

    if(isVisitLandmarkFlagSet(params))
    {
        visitElements(visitor, landmarks);
    }

    if(isVisitLandmarkKeyframeObservationFlagSet(params))
    {
        if(auto graphParams = params.landmarkKeyframeObservationParams.graphParams; graphParams.has_value())
        {
            auto refKeyframe = getKeyframeById(graphParams->refKeyframeId);
            visitNeighbours(visitor, graphParams.value(), refKeyframe);
        }
        else
        {
            visitElements(visitor, indexedObservations);
        }
    }
}

std::shared_ptr<slam3d::Keyframe> BasicMap::getKeyframeById(Id keyframeId)
{
    // TODO: Should keyframes be boost::multiidnex?
    auto foundIt = std::find_if(std::begin(keyframes), std::end(keyframes),
                                [keyframeId](const std::shared_ptr<slam3d::Keyframe>& keyframe)
                                { return keyframeId == keyframe->id; });

    return foundIt == std::end(keyframes) ? nullptr : *foundIt;
}

bool BasicMap::hasKeyframe(std::shared_ptr<slam3d::Keyframe> keyframe) const
{
    return keyframes.find(keyframe) != std::end(keyframes);
}

void BasicMap::addKeyframe(std::shared_ptr<slam3d::Keyframe> keyframe)
{
    keyframes.emplace(std::move(keyframe));
}

void BasicMap::removeKeyframe(std::shared_ptr<slam3d::Keyframe> keyframe)
{
    keyframes.erase(keyframe);
}

void BasicMap::addObservation(
    const LandmarkKeyframeObservation<slam3d::SensorState, slam3d::LandmarkState, rgbd::RgbdKeypoint>& observation)
{
    indexedObservations.insert(observation);
}

bool BasicMap::hasLandmark(std::shared_ptr<slam3d::Landmark> landmark) const
{
    return landmarks.count(landmark) > 0;
}

void BasicMap::addLandmark(std::shared_ptr<slam3d::Landmark> landmark)
{
    landmarks.emplace(std::move(landmark));
}

void BasicMap::removeLandmark(std::shared_ptr<slam3d::Landmark> landmark)
{
    landmarks.erase(landmark);
}

void BasicMap::updateCovisibility(
    std::shared_ptr<Keyframe<slam3d::SensorState>> newKeyframe,
    const std::vector<LandmarkObservation<slam3d::LandmarkState, rgbd::RgbdKeypoint>>& landmarkObservations)
{
    std::for_each(std::cbegin(landmarkObservations), std::cend(landmarkObservations),
                  [&landmarkIndex = indexedObservations.get<1>(), &neighbours = neighbours,
                   &newKeyframe](const LandmarkObservation<slam3d::LandmarkState, rgbd::RgbdKeypoint>& observation)
                  {
                      auto [foundIterator, endIterator] = landmarkIndex.equal_range(observation.landmark);

                      std::for_each(foundIterator, endIterator,
                                    [&neighbours, &newKeyframe](
                                        const LandmarkKeyframeObservation<slam3d::SensorState, slam3d::LandmarkState,
                                                                          rgbd::RgbdKeypoint>& observation)
                                    {
                                        if(newKeyframe != observation.keyframe)
                                        {
                                            neighbours[newKeyframe].insert(observation.keyframe);
                                            neighbours[observation.keyframe].insert(newKeyframe);
                                        };
                                    });
                  });
}

void BasicMap::visitNeighbours(IMapVisitor<slam3d::SensorState, slam3d::LandmarkState, rgbd::RgbdKeypoint>& visitor,
                               const GraphBasedParams& graph, std::shared_ptr<slam3d::Keyframe> refKeyframe)
{
    const auto keyframesToVisit = getNeighbourKeyframes(graph, refKeyframe);

    std::for_each(
        std::begin(keyframesToVisit), std::end(keyframesToVisit),
        [&visitor, &keyframeIndex = indexedObservations.get<0>()](const std::shared_ptr<slam3d::Keyframe>& keyframe)
        {
            auto [foundIterator, endIterator] = keyframeIndex.equal_range(keyframe);

            std::for_each(foundIterator, endIterator,
                          [&visitor](const LandmarkKeyframeObservation<slam3d::SensorState, slam3d::LandmarkState,
                                                                       rgbd::RgbdKeypoint>& observation)
                          { visitor.visit(observation); });
        });
}

std::unordered_set<std::shared_ptr<slam3d::Keyframe>>
BasicMap::getNeighbourKeyframes(const GraphBasedParams& graph, std::shared_ptr<slam3d::Keyframe> refKeyframe)
{
    std::unordered_set<std::shared_ptr<slam3d::Keyframe>> result;
    std::queue<std::pair<std::shared_ptr<slam3d::Keyframe>, int>> neighboursQueue;
    neighboursQueue.push(std::make_pair(refKeyframe, 0));

    while(!neighboursQueue.empty())
    {
        const auto [currentKeyframe, level] = neighboursQueue.front();
        neighboursQueue.pop();

        result.insert(currentKeyframe);

        auto neighboursFoundIt = neighbours.find(currentKeyframe);
        if(level <= graph.deepLevel && neighboursFoundIt != std::end(neighbours))
        {
            for(const auto& neighbourKeyframe : neighboursFoundIt->second)
            {
                const auto notAlreadyVisited = result.find(neighbourKeyframe) == std::end(result);

                if(notAlreadyVisited)
                    neighboursQueue.push(std::make_pair(neighbourKeyframe, level + 1));
            }
        }
    }

    return result;
}

} // namespace mslam
