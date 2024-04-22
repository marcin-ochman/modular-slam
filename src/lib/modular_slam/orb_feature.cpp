#include "modular_slam/orb_feature.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <boost/polymorphic_cast.hpp>
#include <cstdint>
#include <iterator>

#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <spdlog/spdlog.h>
#include <vector>

namespace mslam
{

static constexpr auto descriptorLength = 32;

class OrbOpenCvDetector::Pimpl
{
  public:
    std::vector<OrbKeypoint> detect(const RgbFrame& sensorData);

  private:
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create(1000);
};

std::vector<OrbKeypoint> OrbOpenCvDetector::detect(const RgbFrame& sensorData)
{
    return pimpl->detect(sensorData);
}

std::vector<OrbKeypoint> OrbOpenCvDetector::Pimpl::detect(const RgbFrame& sensorData)
{
    auto gray = toGrayScale(sensorData);
    cv::Mat cvGray{gray.size.height, gray.size.width, CV_8UC1, const_cast<std::uint8_t*>(gray.data.data())};
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    detector->detectAndCompute(cvGray, cv::noArray(), keypoints, descriptors);

    std::vector<OrbKeypoint> result;
    keypoints.reserve(static_cast<std::size_t>(descriptors.rows));

    cv::Mat cvGrayWithKeypoints;
    cvGray.copyTo(cvGrayWithKeypoints);
    cv::drawKeypoints(cvGray, keypoints, cvGrayWithKeypoints);

    cv::imshow("keypoins", cvGrayWithKeypoints);
    cv::pollKey();

    for(auto i = 0; i < descriptors.rows; ++i)
    {
        OrbKeypoint keypoint;
        const auto& cvKeypoint = keypoints[static_cast<std::size_t>(i)];
        const auto* descriptor = descriptors.ptr<std::uint8_t>(i);
        keypoint.keypoint.coordinates = Vector2{cvKeypoint.pt.x, cvKeypoint.pt.y};

        std::copy(descriptor, descriptor + descriptorLength, std::begin(keypoint.descriptor));

        result.push_back(keypoint);
    }

    return result;
}

OrbOpenCvDetector::OrbOpenCvDetector()
{
    pimpl = std::make_unique<Pimpl>();
}

OrbOpenCvDetector::~OrbOpenCvDetector() = default;

class OrbOpenCvMatcher::Pimpl
{
  public:
    std::vector<DescriptorMatch> match(const std::vector<OrbKeypoint>& firstDescriptors,
                                       const std::vector<OrbKeypoint>& secondDescriptors);

  private:
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
};

std::vector<DescriptorMatch> OrbOpenCvMatcher::Pimpl::match(const std::vector<OrbKeypoint>& fromDescriptors,
                                                            const std::vector<OrbKeypoint>& toDescriptors)
{
    std::vector<std::vector<cv::DMatch>> cvMatches;
    cv::Mat fromCvDescriptors(static_cast<int>(fromDescriptors.size()), descriptorLength, CV_8UC1,
                              const_cast<std::uint8_t*>(&fromDescriptors[0].descriptor[0]), sizeof(fromDescriptors[0]));
    cv::Mat toCvDescriptors(static_cast<int>(toDescriptors.size()), descriptorLength, CV_8UC1,
                            const_cast<std::uint8_t*>(&toDescriptors[0].descriptor[0]), sizeof(toDescriptors[0]));

    // TODO: optimize memory allocation
    cv::Mat copyFrom = fromCvDescriptors.clone();
    cv::Mat copyTo = toCvDescriptors.clone();
    matcher->knnMatch(copyTo, copyFrom, cvMatches, 2);

    std::vector<cv::DMatch> goodMatches;
    for(const auto& match : cvMatches)
    {
        if(match[0].distance < 0.7 * match[1].distance)
        {
            goodMatches.push_back(match[0]);
            // spdlog::info("Distance {}", match[0].distance);
        }
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

OrbOpenCvMatcher::OrbOpenCvMatcher()
{
    pimpl = std::make_unique<Pimpl>();
}

std::vector<DescriptorMatch> OrbOpenCvMatcher::match(const std::vector<OrbKeypoint>& fromDescriptors,
                                                     const std::vector<OrbKeypoint>& toDescriptors)
{
    return pimpl->match(fromDescriptors, toDescriptors);
}

OrbOpenCvMatcher::~OrbOpenCvMatcher() = default;

} // namespace mslam
