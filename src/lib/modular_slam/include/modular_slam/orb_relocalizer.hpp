#ifndef MSLAM_ORB_RELOCALIZER_HPP_
#define MSLAM_ORB_RELOCALIZER_HPP_

#include "modular_slam/relocalizer.hpp"
#include "modular_slam/types/slam3d_types.hpp"
#include <cstdint>

namespace mslam
{

class OrbRelocalizer : public IRelocalizer<slam3d::SensorState, std::uint8_t, 32>
{
  public:
    OrbRelocalizer();
    std::vector<std::shared_ptr<Keyframe<slam3d::SensorState>>>
    relocalize(const std::vector<KeypointDescriptor<std::uint8_t, 32>>& keypoints) override;
    void addKeyframe(std::shared_ptr<Keyframe<slam3d::SensorState>> keyframe,
                     const std::vector<KeypointDescriptor<std::uint8_t, 32>>& keypoints) override;
    void removeKeyframe(std::shared_ptr<Keyframe<slam3d::SensorState>> keyframe) override;

    virtual ~OrbRelocalizer();

  private:
    struct OrbRelocalizerPImpl;
    std::unique_ptr<OrbRelocalizerPImpl> pimpl;
};
} // namespace mslam

#endif // MSLAM_ORB_RELOCALIZER_HPP_
