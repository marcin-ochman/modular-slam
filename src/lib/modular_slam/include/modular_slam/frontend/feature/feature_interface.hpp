#ifndef MSLAM_FEATURE_INTERFACE_HPP_
#define MSLAM_FEATURE_INTERFACE_HPP_

#include "modular_slam/types/landmark.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <boost/core/span.hpp>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

namespace mslam
{

struct Keypoint
{
    Id id;
    Vector2 coordinates;
};

template <typename DescriptorType, int Length = 32>
struct KeypointDescriptor
{
    Keypoint keypoint;
    std::array<DescriptorType, Length> descriptor;
};

struct KeypointMatch
{
    Keypoint refKeypoint;
    Keypoint matchedKeypoint;
};

struct DescriptorMatch
{
    std::size_t fromIndex;
    std::size_t toIndex;
};

template <typename LandmarkStateType>
struct KeypointLandmarkMatch
{
    KeypointMatch match;
    std::shared_ptr<Landmark<LandmarkStateType>> landmark;
};

template <typename SensorData, typename DescriptorType, int Length>
class IFeatureDetector
{
  public:
    virtual std::vector<KeypointDescriptor<DescriptorType, Length>> detect(const SensorData& sensorData) = 0;
    virtual ~IFeatureDetector() {}
};

template <typename DescriptorType, int Length>
class IFeatureMatcher
{
  public:
    virtual std::vector<DescriptorMatch>
    match(const std::vector<KeypointDescriptor<DescriptorType, Length>>& firstDescriptors,
          const std::vector<KeypointDescriptor<DescriptorType, Length>>& secondDescriptors) = 0;
    virtual ~IFeatureMatcher() {}
};
} // namespace mslam

#endif // MSLAM_FEATURE_INTERFACE_HPP_
