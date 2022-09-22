#include "modular_slam/orb_feature.hpp"
#include "modular_slam/feature_interface.hpp"
#include "modular_slam/rgb_frame.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <boost/polymorphic_cast.hpp>
#include <opencv2/core/hal/interface.h>

namespace mslam
{

std::unique_ptr<FeatureInterface<Eigen::Vector2f>> OrbFeatureDetector::detect(const RgbFrame& sensorData)
{
    auto gray = toGrayScale(sensorData);

    cv::Mat cvGray{gray.size.height, gray.size.width, CV_8UC1, const_cast<std::uint8_t*>(gray.data.data())};

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    detector->detectAndCompute(cvGray, cv::Mat(), keypoints, descriptors);
    descriptors.convertTo(descriptors, CV_32F);

    return OrbFeature::create(descriptors, keypoints);
}

std::vector<KeypointMatch<Eigen::Vector2f>> OrbFeature::match(FeatureInterface<Eigen::Vector2f>& features)
{
    const auto& orbFeature = boost::polymorphic_downcast<OrbFeature&>(features);

    std::vector<std::vector<cv::DMatch>> matches;
    matcher->knnMatch(descriptors, orbFeature.descriptors, matches, 2);

    std::vector<cv::DMatch> goodMatches;
    for(size_t i = 0; i < matches.size(); i++)
    {
        if(matches[i][0].distance < 0.7 * matches[i][1].distance)
            goodMatches.push_back(matches[i][0]);
    }

    std::vector<KeypointMatch<Eigen::Vector2f>> matchedKeypoints;
    std::transform(std::begin(goodMatches), std::end(goodMatches), std::back_inserter(matchedKeypoints),
                   [&keypoints = orbFeature.keypoints, &refKeypoints = keypoints](const cv::DMatch& match)
                   {
                       const auto& cvKeypoint = keypoints[match.trainIdx];
                       const auto& cvRefKeypoint = refKeypoints[match.queryIdx];

                       KeypointMatch<Eigen::Vector2f> keypointMatch;
                       keypointMatch.matchedKeypoint.id = match.trainIdx;
                       keypointMatch.matchedKeypoint.coordinates = {cvKeypoint.pt.x, cvKeypoint.pt.y};

                       keypointMatch.refKeypoint.id = match.queryIdx;
                       keypointMatch.refKeypoint.coordinates = {cvRefKeypoint.pt.x, cvRefKeypoint.pt.y};

                       return keypointMatch;
                   });

    return matchedKeypoints;
}

std::unique_ptr<OrbFeature> OrbFeature::create(cv::Mat descriptors, const std::vector<cv::KeyPoint>& keypoints)
{
    auto detector = std::make_unique<OrbFeature>();

    detector->descriptors = descriptors;
    detector->keypoints = keypoints;

    return detector;
}

std::vector<Keypoint<Eigen::Vector2f>> OrbFeature::getKeypoints()
{
    std::vector<Keypoint<Eigen::Vector2f>> result;

    Id id = 0;
    for(const auto& cvKeypoint : keypoints)
    {
        Keypoint<Eigen::Vector2f> keypoint = {id++, {cvKeypoint.pt.x, cvKeypoint.pt.y}};
        result.push_back(keypoint);
    }

    return result;
}

} // namespace mslam
