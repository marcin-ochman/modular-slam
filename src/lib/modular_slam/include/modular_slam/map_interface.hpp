#ifndef MSLAM_MAP_INTERFACE_HPP_
#define MSLAM_MAP_INTERFACE_HPP_

#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/keyframe.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/slam_component.hpp"

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

struct MapVisitingParams
{
    std::optional<float> landmarkRadius;
    std::optional<float> keyframeRadius;
};

template <typename SensorStateType, typename LandmarkStateType>
class IMapVisitor
{
  public:
    virtual void visit(std::shared_ptr<Landmark<LandmarkStateType>> landmark) {}
    virtual void visit(std::shared_ptr<Keyframe<SensorStateType>> keyframe) {}
    virtual ~IMapVisitor() = default;
};

template <typename SensorStateType, typename LandmarkStateType>
class IMap : public SlamComponent
{
  public:
    using FrontendOutputType = FrontendOutput<SensorStateType, LandmarkStateType>;

    virtual void update(const std::shared_ptr<FrontendOutputType> constraints) = 0;
    virtual void visit(IMapVisitor<SensorStateType, LandmarkStateType>& visitor,
                       const MapVisitingParams& params = {}) = 0;
};

} // namespace mslam

#endif // MAP_HPP_
