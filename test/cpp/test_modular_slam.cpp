#include <catch2/catch_test_macros.hpp>

#include "modular_slam/slam/slam_builder.hpp"

SCENARIO("SlamBuilder creates a new SLAM system")
{
    auto builder = mslam::SlamBuilder();

    auto slam = builder.build();

    REQUIRE(slam.has_value());
}
