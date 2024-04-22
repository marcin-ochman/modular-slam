#ifndef MSLAM_RELOCALIZER_HPP_
#define MSLAM_RELOCALIZER_HPP_

#include "modular_slam/frontend/feature/feature_interface.hpp"
#include "modular_slam/types/keyframe.hpp"

#include <memory>

namespace mslam
{
template <typename StateType, typename DescriptorType, int DescriptorLength>
class IRelocalizer
{
  public:
    virtual std::vector<std::shared_ptr<Keyframe<StateType>>>
    relocalize(const std::vector<KeypointDescriptor<DescriptorType>>& keypoints) = 0;
    virtual void addKeyframe(std::shared_ptr<Keyframe<StateType>> keyframe,
                             const std::vector<KeypointDescriptor<DescriptorType>>& keypoints) = 0;
    virtual void removeKeyframe(std::shared_ptr<Keyframe<StateType>> keyframe) = 0;
};
} // namespace mslam

#endif // MSLAM_RELOCALIZER_HPP_
