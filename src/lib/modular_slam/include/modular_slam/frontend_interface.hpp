#ifndef MSLAM_FRONTEND_INTERFACE_HPP_
#define MSLAM_FRONTEND_INTERFACE_HPP_

#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/slam_component.hpp"
#include <memory>

namespace mslam
{

template <typename LandmarkStateType>
class LandmarkVisitor
{
  public:
    virtual void visit(std::shared_ptr<Landmark<LandmarkStateType>>&) = 0;
};

template <typename KeyframeStateType>
class KeyframeVisitor
{
  public:
    virtual void visit(std::shared_ptr<Keyframe<KeyframeStateType>>&) = 0;
};

template <typename SensorDataType, typename SensorStateType, typename LandmarkStateType>
class FrontendInterface : public SlamComponent
{
  public:
    using Constraints = ConstraintsInterface<SensorStateType, LandmarkStateType>;

    virtual std::shared_ptr<Constraints> prepareConstraints(const SensorDataType& sensorData) = 0;
    virtual bool update(const Constraints& /*constraints*/) { return true; }

    // TODO: remove it!
    virtual void visitLandmarks(LandmarkVisitor<LandmarkStateType>&) {}
    virtual void visitKeyframes(KeyframeVisitor<SensorStateType>&) {}

    virtual ~FrontendInterface() {}
};

} // namespace mslam

#endif // MSLAM_FRONTEND_INTERFACE_HPP_
