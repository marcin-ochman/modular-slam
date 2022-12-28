#ifndef MSLAM_FEATURE_INTERFACE_HPP_
#define MSLAM_FEATURE_INTERFACE_HPP_

#include "modular_slam/landmark.hpp"
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

template <typename CoordinatesType>
struct Keypoint
{
    Id id;
    CoordinatesType coordinates;
};

template <typename CoordinatesType, typename DescriptorType, int Length = 32>
struct KeypointDescriptor
{
    Keypoint<CoordinatesType> keypoint;
    std::array<DescriptorType, Length> descriptor;
};

template <typename CoordinatesType>
struct KeypointMatch
{
    Keypoint<CoordinatesType> refKeypoint;
    Keypoint<CoordinatesType> matchedKeypoint;
};

struct DescriptorMatch
{
    std::size_t fromIndex;
    std::size_t toIndex;
};

template <typename CoordinatesType, typename LandmarkStateType>
struct KeypointLandmarkMatch
{
    KeypointMatch<CoordinatesType> match;
    std::shared_ptr<Landmark<LandmarkStateType>> landmark;
};

template <typename SensorData, typename KeypointCoordinatesType, typename DescriptorType, int Length>
class IFeatureDetector
{
  public:
    virtual std::vector<KeypointDescriptor<KeypointCoordinatesType, DescriptorType, Length>>
    detect(const SensorData& sensorData) = 0;
    virtual ~IFeatureDetector() {}
};

template <typename KeypointCoordinatesType, typename DescriptorType, int Length>
class IFeatureMatcher
{
  public:
    virtual std::vector<DescriptorMatch> match(
        const std::vector<KeypointDescriptor<KeypointCoordinatesType, DescriptorType, Length>>& firstDescriptors,
        const std::vector<KeypointDescriptor<KeypointCoordinatesType, DescriptorType, Length>>& secondDescriptors) = 0;
    virtual ~IFeatureMatcher() {}
};
} // namespace mslam

#endif // MSLAM_FEATURE_INTERFACE_HPP_
