#include "modular_slam/feature_interface.hpp"

namespace mslam
{

std::vector<Keypoint<CoordinatesType>> FeatureInterface::matchLandmarks(FeatureInterface& features) {}

void bindLandmark(const Keypoint<CoordinatesType>& keypoint,
                  const std::shared_ptr<Landmark<LandmarkStateType>>& landmark);

} // namespace mslam
