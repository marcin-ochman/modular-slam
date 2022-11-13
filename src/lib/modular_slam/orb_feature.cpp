#include "modular_slam/orb_feature.hpp"
#include "modular_slam/feature_interface.hpp"
#include "modular_slam/rgb_frame.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <boost/polymorphic_cast.hpp>
#include <iterator>
#include <opencv2/core/hal/interface.h>

#include <opencv2/core/types.hpp>
#include <spdlog/spdlog.h>

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

std::vector<KeypointMatch<Eigen::Vector2f>> OrbFeature::match(cv::Mat otherDescriptors,
                                                              const std::vector<cv::KeyPoint>& otherKeypoints) const
{
    std::vector<std::vector<cv::DMatch>> matches;
    matcher->knnMatch(cvDescriptors, otherDescriptors, matches, 2);

    std::vector<cv::DMatch> goodMatches;
    for(size_t i = 0; i < matches.size(); i++)
    {
        if(matches[i][0].distance < 0.7 * matches[i][1].distance)
            goodMatches.push_back(matches[i][0]);
    }

    std::vector<KeypointMatch<Eigen::Vector2f>> matchedKeypoints;
    std::transform(std::begin(goodMatches), std::end(goodMatches), std::back_inserter(matchedKeypoints),
                   [&keypoints = otherKeypoints, &refKeypoints = cvKeypoints](const cv::DMatch& match)
                   {
                       const auto& cvKeypoint = keypoints[static_cast<std::size_t>(match.trainIdx)];
                       const auto& cvRefKeypoint = refKeypoints[static_cast<std::size_t>(match.queryIdx)];

                       KeypointMatch<Eigen::Vector2f> keypointMatch;
                       keypointMatch.matchedKeypoint.id = static_cast<Id>(match.trainIdx);
                       keypointMatch.matchedKeypoint.coordinates = {cvKeypoint.pt.x, cvKeypoint.pt.y};

                       keypointMatch.refKeypoint.id = static_cast<Id>(match.queryIdx);
                       keypointMatch.refKeypoint.coordinates = {cvRefKeypoint.pt.x, cvRefKeypoint.pt.y};

                       return keypointMatch;
                   });

    return matchedKeypoints;
}

std::vector<KeypointMatch<Eigen::Vector2f>> OrbFeature::match(FeatureInterface<Eigen::Vector2f>& features) const
{
    const auto& orbFeature = boost::polymorphic_downcast<OrbFeature&>(features);

    return match(orbFeature.cvDescriptors, orbFeature.cvKeypoints);
}

std::vector<DescriptorMatch> OrbFeature::match(boost::span<const Descriptor> descriptors) const
{
    if(descriptors.size() == 0)
        return {};

    std::vector<std::vector<cv::DMatch>> matches;
    const int rows = static_cast<int>(matches.size());
    const int columns = static_cast<int>(descriptors[0].descriptor.size());
    cv::Mat cvOtherDescriptors(rows, columns, CV_32F);

    for(std::size_t i = 0; i < descriptors.size(); ++i)
    {
        auto rowPtr = cvOtherDescriptors.ptr<float>(static_cast<int>(i));
        const auto& currentDescriptor = descriptors[i].descriptor;

        std::copy(std::begin(currentDescriptor), std::end(currentDescriptor), rowPtr);
    }

    matcher->knnMatch(cvDescriptors, cvOtherDescriptors, matches, 2);

    std::vector<cv::DMatch> goodMatches;
    for(size_t i = 0; i < matches.size(); i++)
    {
        if(matches[i][0].distance < 0.7 * matches[i][1].distance)
            goodMatches.push_back(matches[i][0]);
    }

    std::vector<DescriptorMatch> descriptorMatches;

    std::transform(std::begin(goodMatches), std::end(goodMatches), std::back_inserter(descriptorMatches),
                   [](const cv::DMatch& match)
                   {
                       DescriptorMatch descriptorMatch;

                       descriptorMatch.toIndex = static_cast<std::size_t>(match.trainIdx);
                       descriptorMatch.fromIndex = static_cast<std::size_t>(match.queryIdx);

                       return descriptorMatch;
                   });

    return descriptorMatches;
}

std::vector<KeypointMatch<Eigen::Vector2f>> OrbFeature::match(boost::span<const Keypoint<Eigen::Vector2f>> keypoints,
                                                              boost::span<const Descriptor> descriptors) const
{
    const auto rows = static_cast<int>(descriptors.size());
    if(rows == 0)
        return {};

    const auto cols = descriptors[0].descriptor.size();
    cv::Mat refCvDescriptors(rows, static_cast<int>(cols), CV_32F);

    auto i = 0;
    for(const auto& descriptor : descriptors)
    {
        auto ptr = refCvDescriptors.ptr<float>(i);
        std::copy(std::begin(descriptor.descriptor), std::end(descriptor.descriptor), ptr);
    }

    std::vector<cv::KeyPoint> refKeypoints;
    refKeypoints.reserve(keypoints.size());

    std::transform(std::begin(keypoints), std::end(keypoints), std::back_inserter(refKeypoints),
                   [](const Keypoint<Eigen::Vector2f>& keypoint)
                   { return cv::KeyPoint(keypoint.coordinates.x(), keypoint.coordinates.y(), 1); });

    return match(refCvDescriptors, refKeypoints);
}

std::unique_ptr<OrbFeature> OrbFeature::create(cv::Mat descriptors, const std::vector<cv::KeyPoint>& keypoints)
{
    auto detector = std::make_unique<OrbFeature>();

    detector->cvDescriptors = descriptors;
    detector->cvKeypoints = keypoints;

    return detector;
}

std::vector<Keypoint<Eigen::Vector2f>> OrbFeature::keypoints() const
{
    std::vector<Keypoint<Eigen::Vector2f>> result;

    Id id = 0;
    for(const auto& cvKeypoint : cvKeypoints)
    {
        Keypoint<Eigen::Vector2f> keypoint = {id++, {cvKeypoint.pt.x, cvKeypoint.pt.y}};
        result.push_back(keypoint);
    }

    return result;
}

std::vector<Descriptor> OrbFeature::descriptors() const
{
    std::vector<Descriptor> result;
    result.reserve(static_cast<std::size_t>(cvDescriptors.rows));

    Id id = 0;
    for(auto row = 0; row < cvDescriptors.rows; ++row, ++id)
    {
        Descriptor descriptor = {id, boost::span<const float>(cvDescriptors.ptr<const float>(row),
                                                              static_cast<std::size_t>(cvDescriptors.cols))};
        result.push_back(descriptor);
    }

    return result;
}

} // namespace mslam
