#include "modular_slam/odometry.hpp"

#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

namespace mslam::odom
{

using namespace cv;

using Matches = std::vector<std::vector<DMatch>>;

std::optional<Transform> OrbBasicOdometry::estimatePose(const OrbKeypoints& keypoints)
{
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);

    if(previousKeypoints.size() == 0)
        return std::nullopt;

    Matches matches;

    matcher->knnMatch(keyp, descriptors2, knnMatches, 2);
}
} // namespace mslam::odom
