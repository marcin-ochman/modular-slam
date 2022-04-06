#define CATCH_CONFIG_MAIN

#include "modular_slam/odometry.hpp"
#include <catch2/catch.hpp>

SCENARIO("Testing OrbBasic")
{
    GIVEN("OrbBasicOdometry")
    {
        mslam::odom::OrbBasicOdometry orbOdom;

        WHEN("Called for the first time")
        {
            const std::vector<cv::KeyPoint> keypoints;
            auto transform = orbOdom.estimatePose(keypoints);
            REQUIRE(transform == std::nullopt);
        }

        WHEN("Called for the second time")
        {
            const std::vector<cv::KeyPoint> keypoints;
            auto transform = orbOdom.estimatePose(keypoints);

            REQUIRE(transform != std::nullopt);
        }
    }
}
