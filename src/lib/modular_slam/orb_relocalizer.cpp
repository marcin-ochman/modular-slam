#include "modular_slam/orb_relocalizer.hpp"

#include "DBoW3/Database.h"
#include "DBoW3/QueryResults.h"
#include "DBoW3/Vocabulary.h"
#include <memory>
#include <opencv2/core/mat.hpp>

namespace mslam
{

struct OrbRelocalizer::OrbRelocalizerPImpl
{
    OrbRelocalizerPImpl();
    std::vector<std::shared_ptr<Keyframe<slam3d::SensorState>>>
    relocalize(const std::vector<KeypointDescriptor<float, 32>>& keypoints);
    void addKeyframe(std::shared_ptr<Keyframe<slam3d::SensorState>> keyframe,
                     const std::vector<KeypointDescriptor<float, 32>>& keypoints);
    void removeKeyframe(std::shared_ptr<Keyframe<slam3d::SensorState>> keyframe);
    DBoW3::Database database;

    std::unordered_map<Id, DBoW3::EntryId> keyframeIdToEntry;
};

OrbRelocalizer::OrbRelocalizerPImpl::OrbRelocalizerPImpl()
{
    DBoW3::Vocabulary vocabulary("orbvoc.dbow3");
    database = DBoW3::Database(vocabulary, false, 0);
}

std::vector<std::shared_ptr<Keyframe<slam3d::SensorState>>>
OrbRelocalizer::OrbRelocalizerPImpl::relocalize(const std::vector<KeypointDescriptor<float, 32>>& keypoints)
{
    return {};
}

void OrbRelocalizer::OrbRelocalizerPImpl::addKeyframe(std::shared_ptr<Keyframe<slam3d::SensorState>> keyframe,
                                                      const std::vector<KeypointDescriptor<float, 32>>& keypoints)
{
    assert(keypoints.size() > 0);

    cv::Mat features(keypoints.size(), 32, CV_32F, (void*)keypoints[0].descriptor.data(),
                     static_cast<std::size_t>(sizeof(keypoints[0])));

    const auto entryId = database.add(features);

    keyframeIdToEntry.insert({keyframe->id, entryId});
}

void OrbRelocalizer::OrbRelocalizerPImpl::removeKeyframe(std::shared_ptr<Keyframe<slam3d::SensorState>> keyframe)
{
    // TODO
}

OrbRelocalizer::~OrbRelocalizer() = default;

OrbRelocalizer::OrbRelocalizer()
{
    pimpl = std::make_unique<OrbRelocalizerPImpl>();
}

std::vector<std::shared_ptr<Keyframe<slam3d::SensorState>>>
OrbRelocalizer::relocalize(const std::vector<KeypointDescriptor<float, 32>>& keypoints)
{
    return pimpl->relocalize(keypoints);
}

void OrbRelocalizer::addKeyframe(std::shared_ptr<Keyframe<slam3d::SensorState>> keyframe,
                                 const std::vector<KeypointDescriptor<float, 32>>& keypoints)
{
    return pimpl->addKeyframe(keyframe, keypoints);
}

void OrbRelocalizer::removeKeyframe(std::shared_ptr<Keyframe<slam3d::SensorState>> keyframe)
{
    pimpl->removeKeyframe(keyframe);
}

} // namespace mslam
