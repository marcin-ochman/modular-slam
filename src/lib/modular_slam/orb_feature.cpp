#include "modular_slam/orb_feature.hpp"
#include "modular_slam/feature_interface.hpp"
#include "modular_slam/rgb_frame.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <boost/polymorphic_cast.hpp>
#include <iterator>
#include <opencv2/core/hal/interface.h>

#include <opencv2/core/types.hpp>
#include <spdlog/spdlog.h>
#include <vector>

namespace mslam
{

static constexpr auto descriptorLength = 32;

std::vector<OrbKeypoint> OrbOpenCvDetector::detect(const RgbFrame& sensorData)
{
    auto gray = toGrayScale(sensorData);
    cv::Mat cvGray{gray.size.height, gray.size.width, CV_8UC1, const_cast<std::uint8_t*>(gray.data.data())};
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    detector->detectAndCompute(cvGray, cv::Mat(), keypoints, descriptors);
    descriptors.convertTo(descriptors, CV_32F);

    std::vector<OrbKeypoint> result;
    keypoints.reserve(descriptors.rows);

    for(auto i = 0; i < descriptors.rows; ++i)
    {
        OrbKeypoint keypoint;
        const auto& cvKeypoint = keypoints[i];
        const auto* descriptor = descriptors.ptr<float>(i);
        keypoint.keypoint.coordinates = Eigen::Vector2f{cvKeypoint.pt.x, cvKeypoint.pt.y};

        std::copy(descriptor, descriptor + descriptorLength, std::begin(keypoint.descriptor));

        result.push_back(keypoint);
    }

    return result;
}

std::vector<DescriptorMatch> OrbOpenCvMatcher::match(const std::vector<OrbKeypoint>& fromDescriptors,
                                                     const std::vector<OrbKeypoint>& toDescriptors)
{
    std::vector<std::vector<cv::DMatch>> cvMatches;
    cv::Mat fromCvDescriptors(static_cast<int>(fromDescriptors.size()), descriptorLength, CV_32F,
                              const_cast<float*>(&fromDescriptors[0].descriptor[0]), sizeof(fromDescriptors[0]));
    cv::Mat toCvDescriptors(static_cast<int>(toDescriptors.size()), descriptorLength, CV_32F,
                            const_cast<float*>(&toDescriptors[0].descriptor[0]), sizeof(toDescriptors[0]));

    // TODO: optimize memory allocation
    cv::Mat copyFrom = fromCvDescriptors.clone();
    cv::Mat copyTo = toCvDescriptors.clone();
    matcher->knnMatch(copyTo, copyFrom, cvMatches, 2);

    std::vector<cv::DMatch> goodMatches;
    for(const auto& match : cvMatches)
    {
        if(match[0].distance < 0.7 * match[1].distance)
            goodMatches.push_back(match[0]);
    }

    std::vector<DescriptorMatch> matches;
    matches.reserve(goodMatches.size());

    std::transform(
        std::begin(goodMatches), std::end(goodMatches), std::back_inserter(matches),
        [](const cv::DMatch& match) {
            return DescriptorMatch{static_cast<std::size_t>(match.trainIdx), static_cast<std::size_t>(match.queryIdx)};
        });

    return matches;
}

} // namespace mslam
