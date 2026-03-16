#include <catch2/catch_test_macros.hpp>

#include "modular_slam/core/pose.hpp"
#include "modular_slam/core/time_stamp.hpp"
#include "modular_slam/modular_slam.hpp"
#include "modular_slam/sensors/camera/pinhole.hpp"
#include "modular_slam/sensors/camera_frame.hpp"

using CameraFrame = modular_slam::CameraFrame<modular_slam::PinholeCamera<>>;

SCENARIO("SlamBuilder creates a new SLAM system")
{
    GIVEN("SlamBuilder and SLAM components")
    {
        auto builder = modular_slam::SlamBuilder<>::create();

        struct TestFrontend
        {
            using InputData = CameraFrame;
            using OutputFactor = int;

            int process(InputData, const modular_slam::Pose& predictedPose) { return OutputFactor{}; }
        };

        struct TestBackend
        {
        };

        struct TestPropagator
        {
            modular_slam::Pose predict([[maybe_unused]] const modular_slam::Timestamp& timestamp)
            {
                return modular_slam::Pose{};
            }
        };

        struct TestMap
        {
        };

        THEN("Inputs and outputs are simple")
        {
            STATIC_REQUIRE(std::is_trivially_copyable_v<CameraFrame>);
        }

        THEN("SLAM can be created")
        {
            auto vslam = builder.withBackend<TestBackend>()
                             .withPropagator<TestPropagator>()
                             .withFrontend<TestFrontend>()
                             .withMap<TestMap>()
                             .build();

            THEN("SLAM can be executed")
            {
                CameraFrame cameraFrame;

                vslam.feed(cameraFrame);
            }
        }
    }
}

SCENARIO("IsFrontend concept verification fails.", "[templates]")
{
    GIVEN("A frontend that lacks the OutputFactor type")
    {
        struct TestFrontend
        {
            using InputData = int;
            int process(InputData) { return int{}; }
        };

        THEN("The concept should evaluate to false")
        {
            STATIC_REQUIRE_FALSE(modular_slam::frontend::IsFrontend<TestFrontend>);
        }
    }

    GIVEN("A frontend that lacks process()")
    {
        struct TestFrontend
        {
            using OutputFactor = int;
            using InputData = int;
        };

        THEN("The concept should evaluate to false")
        {
            STATIC_REQUIRE_FALSE(modular_slam::frontend::IsFrontend<TestFrontend>);
        }
    }

    GIVEN("A frontend that has Wrong process() return type")
    {
        struct TestFrontend
        {
            using OutputFactor = int;
            using InputData = int;
            void process(InputData) {}
        };

        THEN("The concept should evaluate to false")
        {
            STATIC_REQUIRE_FALSE(modular_slam::frontend::IsFrontend<TestFrontend>);
        }
    }
}

TEST_CASE("IsFrontend concept verification fails. No ", "[templates]")
{
    struct TestFrontend
    {
        using OutputFactor = int;
        using InputData = int;
    };

    STATIC_REQUIRE_FALSE(modular_slam::frontend::IsFrontend<TestFrontend>);
}

TEST_CASE("IsVisualFrontend concept verification.", "[templates]")
{
    struct TestFrontend
    {
        using OutputFactor = int;
        using InputData = modular_slam::CameraFrame<modular_slam::PinholeCamera<>>;
    };

    STATIC_REQUIRE_FALSE(modular_slam::frontend::IsFrontend<TestFrontend>);
}
