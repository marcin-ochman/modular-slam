#define CATCH_CONFIG_MAIN

#include "modular_slam/modular_slam.hpp"
#include <catch2/catch.hpp>

namespace mslam
{

SCENARIO("Loop detection with ORB features")
{
    GIVEN("ORB features and already visited places")
    {
        std::array<mslam::Frame, 3> testFrames = {1, 2, 3};

        LoopDetection* loopDetection = nullptr;

        for(auto& frame : testFrames)
        {
            // REQUIRE(loopDetection->detectLoop());
        }

        WHEN("Loop closure occurs")
        {
            THEN("Algorithm detects loop closure") {}
        }
    }
}
} // namespace mslam
