#ifndef MSLAM_MAP_INTERFACE_HPP_
#define MSLAM_MAP_INTERFACE_HPP_

#include "modular_slam/backend/backend_interface.hpp"
#include "modular_slam/frontend/feature/feature_interface.hpp"
#include "modular_slam/slam_component.hpp"
#include "modular_slam/types/frontend_output.hpp"
#include <any>
#include <ceres/types.h>
#include <limits>
#include <type_traits>

namespace mslam
{

template <typename SensorStateType, typename LandmarkStateType>
class IMapComponentsFactory
{
  public:
    virtual std::shared_ptr<Landmark<LandmarkStateType>> createLandmark() = 0;
    virtual std::shared_ptr<Keyframe<SensorStateType>> createKeyframe() = 0;
    virtual ~IMapComponentsFactory() = default;
};

struct GraphBasedParams
{
    Id refKeyframeId;
    std::size_t deepLevel;
};

struct LandmarkVisitingParams
{
    std::optional<double> radius;
    std::optional<GraphBasedParams> graphParams;
    std::unordered_set<Id> observedByKeyframes;
};

// TODO it should be templated depending on keypoints
struct KeyframeSimilarityParams
{
    std::vector<KeypointDescriptor<float, 32>> keypoints;
    double scoreThreshold;
};

struct KeyframeVisitingParams
{
    // static constexpr auto maxRadius = std::numeric_limits<double>::max();
    // std::optional<double> radius;
    std::optional<KeyframeSimilarityParams> similarity;
};

struct LandmarkKeyframeObservationParams
{
    std::optional<GraphBasedParams> graphParams;
};

enum class MapElementsToVisit : std::uint8_t
{
    None = 0,
    Keyframe = 1,
    Landmark = 2,
    LandmarkKeyframeObservation = 4,

    All = MapElementsToVisit::Keyframe | MapElementsToVisit::Landmark | MapElementsToVisit::LandmarkKeyframeObservation
};

struct MapVisitingParams
{
    LandmarkVisitingParams landmarkParams;
    KeyframeVisitingParams keyframeParams;
    LandmarkKeyframeObservationParams observationParams;
    MapElementsToVisit elementsToVisit = MapElementsToVisit::All;

    // TODO: think about it
    // std::map<std::string, std::any> additionalParams;
};

inline bool isVisitFlagSet(const MapVisitingParams& visitingParams, const MapElementsToVisit elementsToVisit)
{
    using UnderlyingType = std::underlying_type_t<MapElementsToVisit>;
    return (static_cast<UnderlyingType>(visitingParams.elementsToVisit) &
            static_cast<UnderlyingType>(elementsToVisit)) != 0;
}

inline bool isVisitLandmarkFlagSet(const MapVisitingParams& visitingParams)
{
    return isVisitFlagSet(visitingParams, MapElementsToVisit::Landmark);
}

inline bool isVisitKeyframeFlagSet(const MapVisitingParams& visitingParams)
{
    return isVisitFlagSet(visitingParams, MapElementsToVisit::Keyframe);
}

inline bool isVisitLandmarkKeyframeObservationFlagSet(const MapVisitingParams& visitingParams)
{
    return isVisitFlagSet(visitingParams, MapElementsToVisit::LandmarkKeyframeObservation);
}

template <typename SensorStateType, typename LandmarkStateType, typename ObservationType>
class IMapVisitor
{
  public:
    virtual void visit(std::shared_ptr<Landmark<LandmarkStateType>> landmark) {}
    virtual void visit(std::shared_ptr<Keyframe<SensorStateType>> keyframe) {}
    virtual void visit(const LandmarkKeyframeObservation<SensorStateType, LandmarkStateType, ObservationType>&
                           landmarkKeyframeObservation)
    {
    }

    virtual ~IMapVisitor() = default;
};

template <typename SensorStateType, typename LandmarkStateType, typename ObservationType>
class IMap : public SlamComponent
{
  public:
    using FrontendOutputType = FrontendOutput<SensorStateType, LandmarkStateType, ObservationType>;
    using BackendOutputType = BackendOutput<SensorStateType, LandmarkStateType, ObservationType>;
    using VisitorType = IMapVisitor<SensorStateType, LandmarkStateType, ObservationType>;

    virtual void update(const FrontendOutputType& frontendOutput) = 0;
    virtual void update(const BackendOutputType& frontendOutput) = 0;
    virtual void visit(IMapVisitor<SensorStateType, LandmarkStateType, ObservationType>& visitor,
                       const MapVisitingParams& params = {}) = 0;
};

} // namespace mslam

#endif // MAP_HPP_
