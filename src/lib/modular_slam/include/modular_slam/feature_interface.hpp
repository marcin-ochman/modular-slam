#ifndef MSLAM_FEATURE_INTERFACE_HPP_
#define MSLAM_FEATURE_INTERFACE_HPP_

#include "modular_slam/landmark.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>

namespace mslam
{

enum class FeatureType
{
    Orb,
    Sift,
    Surf
};

template <typename CoordinatesType>
struct Keypoint
{
    Id id;
    CoordinatesType coordinates;
};

template <typename CoordinatesType>
struct KeypointMatch
{
    Keypoint<CoordinatesType> refKeypoint;
    Keypoint<CoordinatesType> matchedKeypoint;
};

template <typename CoordinatesType, typename LandmarkStateType>
struct KeypointLandmarkMatch
{
    KeypointMatch<CoordinatesType> match;

    std::shared_ptr<Landmark<LandmarkStateType>> landmark;
};

template <typename CoordinatesType = Eigen::Vector2f, typename LandmarkStateType = Vector3>
class FeatureInterface
{
  public:
    virtual std::vector<KeypointMatch<CoordinatesType>> match(FeatureInterface& features) = 0;
    virtual std::vector<Keypoint<CoordinatesType>> getKeypoints() = 0;
    virtual std::unique_ptr<FeatureInterface> fromIds() { return nullptr; };
    virtual int type() = 0;

    std::vector<KeypointLandmarkMatch<CoordinatesType, LandmarkStateType>> matchLandmarks(FeatureInterface& features);
    void bindLandmark(const Keypoint<CoordinatesType>& keypoint, std::shared_ptr<Landmark<LandmarkStateType>> landmark);

  protected:
    std::map<Id, std::shared_ptr<Landmark<LandmarkStateType>>> landmarks;
};

template <typename SensorData, typename CoordinatesType = Eigen::Vector2f, typename LandmarkStateType = Vector3>
class FeatureDetectorInterface
{
  public:
    virtual std::unique_ptr<FeatureInterface<CoordinatesType, LandmarkStateType>>
    detect(const SensorData& sensorData) = 0;
    virtual ~FeatureDetectorInterface() {}
};

template <typename CoordinatesType, typename LandmarkStateType>
std::vector<KeypointLandmarkMatch<CoordinatesType, LandmarkStateType>>
FeatureInterface<CoordinatesType, LandmarkStateType>::matchLandmarks(FeatureInterface& features)
{
    const auto matchedKeypoints = match(features);
    std::vector<KeypointLandmarkMatch<CoordinatesType, LandmarkStateType>> matches;

    matches.reserve(matchedKeypoints.size());
    std::transform(std::cbegin(matchedKeypoints), std::cend(matchedKeypoints), std::back_inserter(matches),
                   [&landmarks = landmarks](const KeypointMatch<Eigen::Vector2f>& keypointMatch)
                   {
                       KeypointLandmarkMatch<CoordinatesType, LandmarkStateType> match;

                       auto foundLandmarkIt = landmarks.find(keypointMatch.refKeypoint.id);
                       match.match = keypointMatch;
                       match.landmark = foundLandmarkIt == std::end(landmarks) ? nullptr : foundLandmarkIt->second;

                       return match;
                   });

    return matches;
}

template <typename CoordinatesType, typename LandmarkStateType>
void FeatureInterface<CoordinatesType, LandmarkStateType>::bindLandmark(
    const Keypoint<CoordinatesType>& keypoint, std::shared_ptr<Landmark<LandmarkStateType>> landmark)
{
    landmarks[keypoint.id] = landmark;
}

// template <typename SensorData, typename CoordinatesType = Eigen::Vector2f, typename LandmarkStateType = Vector3>
// class FeatureLandmarkInterface
// {
//   public:
//     std::vector<KeypointLandmarkMatch<CoordinatesType, LandmarkStateType>> matchLandmarks(FeatureInterface&
//     features); void bindLandmark(const Keypoint<CoordinatesType>& keypoint,
//     std::shared_ptr<Landmark<LandmarkStateType>> landmark);
// };

} // namespace mslam

#endif // MSLAM_FEATURE_INTERFACE_HPP_
