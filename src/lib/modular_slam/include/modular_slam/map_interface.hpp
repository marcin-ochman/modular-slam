#ifndef MSLAM_MAP_INTERFACE_HPP_
#define MSLAM_MAP_INTERFACE_HPP_

#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/keyframe.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/slam_component.hpp"
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
};

struct KeyframeVisitingParams
{
    static constexpr auto maxRadius = std::numeric_limits<double>::max();
    std::optional<double> radius;
    std::optional<GraphBasedParams> graphParams;
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
    LandmarkKeyframeObservationParams landmarkKeyframeObservationParams;

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

template <typename SensorStateType, typename LandmarkStateType>
class IMapVisitor
{
  public:
    virtual void visit(std::shared_ptr<Landmark<LandmarkStateType>> landmark) {}
    virtual void visit(std::shared_ptr<Keyframe<SensorStateType>> keyframe) {}
    virtual void
    visit(const LandmarkKeyframeObservation<SensorStateType, LandmarkStateType>& landmarkKeyframeObservation)
    {
    }
    virtual ~IMapVisitor() = default;
};

template <typename SensorStateType, typename LandmarkStateType>
class IMap : public SlamComponent
{
  public:
    using FrontendOutputType = FrontendOutput<SensorStateType, LandmarkStateType>;

    virtual void update(const FrontendOutputType& frontendOutput) = 0;
    virtual void visit(IMapVisitor<SensorStateType, LandmarkStateType>& visitor,
                       const MapVisitingParams& params = {}) = 0;
};

} // namespace mslam

#endif // MAP_HPP_
