#ifndef MSLAM_CONSTRAINTS_INTERFACE_HPP_
#define MSLAM_CONSTRAINTS_INTERFACE_HPP_

#include "modular_slam/keyframe.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/observation.hpp"

namespace mslam
{

template <typename SensorStateType, typename LandmarkStateType, typename LandmarkConstraintType = LandmarkStateType>
struct LandmarkConstraint
{
    std::shared_ptr<Keyframe<SensorStateType>> keyframe;
    std::shared_ptr<Landmark<LandmarkStateType>> landmark;
    LandmarkConstraintType constraint;
};

template <typename SensorStateType, typename LandmarkStateType, typename KeyframeConstraintType = SensorStateType>
struct KeyframeConstraint
{
    std::shared_ptr<Keyframe<SensorStateType>> firstKeyframe;
    std::shared_ptr<Keyframe<SensorStateType>> secondKeyframe;
    KeyframeConstraintType constraint;
};

template <typename SensorStateType, typename LandmarkStateType, typename KeyframeConstraintType = SensorStateType,
          typename LandmarkConstraintType = LandmarkStateType>
class ConstraintVisitor
{
  public:
    virtual void visit(const LandmarkConstraint<SensorStateType, LandmarkStateType>& constraint) = 0;
    virtual void visit(const KeyframeConstraint<SensorStateType, LandmarkStateType>& constraint) = 0;
};

template <typename SensorStateType, typename LandmarkStateType, typename KeyframeConstraintType = SensorStateType,
          typename LandmarkConstraintType = LandmarkStateType>
class ConstraintsInterface
{
  public:
    virtual void
    addConstraint(const LandmarkConstraint<SensorStateType, LandmarkStateType, LandmarkConstraintType> constraint) = 0;
    virtual void
    addConstraint(const KeyframeConstraint<SensorStateType, LandmarkStateType, KeyframeConstraintType> constraint) = 0;

    virtual SensorStateType currentState() = 0;
    virtual void visitConstraints(ConstraintVisitor<SensorStateType, LandmarkStateType, KeyframeConstraintType,
                                                    LandmarkConstraintType>& visitor) = 0;

    virtual bool removeKeyframe(const Keyframe<SensorStateType>& keyframe) = 0;
    virtual bool removeLandmark(const Landmark<LandmarkStateType>& landmark) = 0;

    virtual ~ConstraintsInterface() {}
};

} // namespace mslam

#endif // MSLAM_CONSTRAINTS_INTERFACE_HPP_
