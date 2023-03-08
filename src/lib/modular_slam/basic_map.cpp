#include "modular_slam/basic_map.hpp"
#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/map_interface.hpp"
#include "modular_slam/slam3d_types.hpp"
#include <algorithm>
#include <iterator>
#include <queue>

namespace mslam
{

BasicMap::BasicMap() {}

void BasicMap::update(const FrontendOutput<slam3d::SensorState, slam3d::LandmarkState>& frontendOutput)
{
    if(frontendOutput.newKeyframe != nullptr)
    {
        addKeyframe(frontendOutput.newKeyframe);

        std::for_each(std::begin(frontendOutput.newLandmarks), std::end(frontendOutput.newLandmarks),
                      [this](auto& landmark) { addLandmark(landmark); });

        std::for_each(
            std::begin(frontendOutput.landmarkObservations), std::end(frontendOutput.landmarkObservations),
            [this, keyframe = frontendOutput.newKeyframe](const LandmarkObservation<slam3d::LandmarkState>& observation)
            {
                LandmarkKeyframeObservation<slam3d::SensorState, slam3d::LandmarkState> newObservation = {
                    keyframe, observation.landmark, observation.keypoint};

                addObservation(newObservation);
            });

        updateCovisibility(frontendOutput.newKeyframe, frontendOutput.landmarkObservations);
    }
}

template <typename T>
void visitElements(IMapVisitor<slam3d::SensorState, slam3d::LandmarkState>& visitor, T&& container)
{
    std::for_each(std::cbegin(container), std::cend(container),
                  [&visitor](const auto& element) { visitor.visit(element); });
}

void BasicMap::visit(IMapVisitor<slam3d::SensorState, slam3d::LandmarkState>& visitor, const MapVisitingParams& params)
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
            for(const auto& [keyframe, observations] : observations)
            {
                visitElements(visitor, observations);
            }
        }
    }
}

std::shared_ptr<slam3d::Keyframe> BasicMap::getKeyframeById(Id keyframeId)
{
    // using namespace std::placeholders;
    // std::bind(std::equal_to(), keyframeId)
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
    const LandmarkKeyframeObservation<slam3d::SensorState, slam3d::LandmarkState>& observation)
{
    observations[observation.keyframe].push_back(observation);
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

void BasicMap::updateCovisibility(std::shared_ptr<Keyframe<slam3d::SensorState>> newKeyframe,
                                  const std::vector<LandmarkObservation<slam3d::LandmarkState>>& landmarkObservations)
{
    std::for_each(
        std::cbegin(landmarkObservations), std::cend(landmarkObservations),
        [&landmarkIndex = indexedObservations.get<1>(), &neighbours = neighbours,
         &newKeyframe](const LandmarkObservation<slam3d::LandmarkState>& observation)
        {
            auto [foundIterator, endIterator] = landmarkIndex.equal_range(observation.landmark);

            std::for_each(
                foundIterator, endIterator,
                [&neighbours, &newKeyframe](
                    const LandmarkKeyframeObservation<slam3d::SensorState, slam3d::LandmarkState>& observation)
                {
                    if(newKeyframe != observation.keyframe)
                    {
                        neighbours[newKeyframe].insert(observation.keyframe);
                        neighbours[observation.keyframe].insert(newKeyframe);
                    };
                });
        });
}

void BasicMap::visitNeighbours(IMapVisitor<slam3d::SensorState, slam3d::LandmarkState>& visitor,
                               const GraphBasedParams& graph, std::shared_ptr<slam3d::Keyframe> refKeyframe)
{
    std::set<std::shared_ptr<slam3d::Keyframe>> keyframesToVisit = getNeighbourKeyframes(graph, refKeyframe);

    std::for_each(
        std::begin(keyframesToVisit), std::end(keyframesToVisit),
        [&visitor, &keyframeIndex = indexedObservations.get<0>()](const std::shared_ptr<slam3d::Keyframe>& keyframe)
        {
            auto [foundIterator, endIterator] = keyframeIndex.equal_range(keyframe);

            std::for_each(
                foundIterator, endIterator,
                [&visitor](const LandmarkKeyframeObservation<slam3d::SensorState, slam3d::LandmarkState>& observation)
                { visitor.visit(observation); });
        });
}

// void bfs(std::shared_ptr<slam3d::Keyframe> keyframe,
//          const std::map<std::shared_ptr<slam3d::Keyframe>, std::set<std::shared_ptr<slam3d::Keyframe>>>& neighbours,
//          int level, int maxLevel, std::set<std::shared_ptr<slam3d::Keyframe>>& result)
// {
//     if(level > maxLevel)
//         return;

//     result.insert(keyframe);

//     auto neighboursFoundIt = neighbours.find(keyframe);
//     if(neighboursFoundIt == std::end(neighbours))
//         return;

//     for(auto& neighbourKeyframe : neighboursFoundIt->second)
//     {
//         auto foundIt = result.find(neighbourKeyframe);

//         if(foundIt == std::end(result))
//         {
//             bfs(neighbourKeyframe, neighbours, level + 1, maxLevel, result);
//         }
//     }
// }

std::set<std::shared_ptr<slam3d::Keyframe>>
BasicMap::getNeighbourKeyframes(const GraphBasedParams& graph, std::shared_ptr<slam3d::Keyframe> refKeyframe)
{
    std::set<std::shared_ptr<slam3d::Keyframe>> result;

    std::queue<std::pair<std::shared_ptr<slam3d::Keyframe>, int>> queueToVisit;
    queueToVisit.push(std::make_pair(refKeyframe, 0));

    while(!queueToVisit.empty())
    {
        auto [currentKeyframe, level] = queueToVisit.front();
        queueToVisit.pop();

        result.insert(currentKeyframe);

        auto neighboursFoundIt = neighbours.find(currentKeyframe);
        if(level <= graph.deepLevel && neighboursFoundIt != std::end(neighbours))
        {
            for(const auto& neighbourKeyframe : neighboursFoundIt->second)
            {
                if(result.find(neighbourKeyframe) == std::end(result))
                    queueToVisit.push(std::make_pair(neighbourKeyframe, level + 1));
            }
        }
    }

    return result;
}

} // namespace mslam
