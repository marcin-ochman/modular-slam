#ifndef SLAM_STATISTICS_HPP_
#define SLAM_STATISTICS_HPP_

#include <cstddef>

struct SlamStatistics
{
    float msPerFrame;
    std::size_t landmarksCount;
    std::size_t keyframesCount;
    std::size_t observationsCount;
};

#endif // SLAM_STATISTICS_HPP_
