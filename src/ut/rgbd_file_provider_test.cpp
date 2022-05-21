#define CATCH_CONFIG_MAIN

#include "modular_slam/rgbd_file_provider.hpp"

#include <algorithm>
#include <catch2/catch.hpp>
#include <opencv2/imgcodecs.hpp>

namespace mslam
{

namespace fs = std::filesystem;

const fs::path testDataPath = TEST_DATA_DIR;
const fs::path rgbDirPath = testDataPath / "rgb";
const fs::path depthDirPath = testDataPath / "depth";

SCENARIO("Data provider is RgbdFileProvider")
{
    GIVEN("RgbdFileProvider instance with specified paths to rgb and depth data after fetch")
    {
        RgbdFileProvider fileProvider{rgbDirPath, depthDirPath};

        REQUIRE(fileProvider.fetch());

        auto rgbd = fileProvider.recentData();

        THEN("RGB-D depth was read and is valid")
        {
            REQUIRE(rgbd != nullptr);

            auto refMatRgb = cv::imread((rgbDirPath / "0000.png").string());
            auto refMatDepth = cv::imread((depthDirPath / "0000.png").string(), cv::IMREAD_ANYDEPTH);

            REQUIRE(rgbd->rgbData.size() == refMatRgb.size().height * refMatRgb.size().width * refMatRgb.channels());
            REQUIRE(std::equal(rgbd->rgbData.begin(), rgbd->rgbData.end(), refMatRgb.ptr()));

            REQUIRE(rgbd->depthData.size() == refMatDepth.size().height * refMatDepth.size().width);
            REQUIRE(std::equal(rgbd->depthData.begin(), rgbd->depthData.begin() + 307200,
                               refMatDepth.ptr<std::uint16_t>()));
        }

        REQUIRE(fileProvider.fetch());

        auto nextRgbd = fileProvider.recentData();
        THEN("New RGB-D depth was read and is valid")
        {
            REQUIRE(nextRgbd != nullptr);

            auto refMatRgb = cv::imread((rgbDirPath / "0001.png").string());
            auto refMatDepth = cv::imread((depthDirPath / "0001.png").string(), cv::IMREAD_ANYDEPTH);

            REQUIRE(nextRgbd->rgbData.size() ==
                    refMatRgb.size().height * refMatRgb.size().width * refMatRgb.channels());
            REQUIRE(std::equal(nextRgbd->rgbData.begin(), nextRgbd->rgbData.end(), refMatRgb.ptr()));

            REQUIRE(nextRgbd->depthData.size() == refMatDepth.size().height * refMatDepth.size().width);
            REQUIRE(std::equal(nextRgbd->depthData.begin(), nextRgbd->depthData.begin() + 307200,
                               refMatDepth.ptr<std::uint16_t>()));
        }

        REQUIRE_FALSE(fileProvider.fetch());
        auto lastRgbd = fileProvider.recentData();
        THEN("New RGB-D depth was not read and is not valid") { REQUIRE(lastRgbd == nullptr); }
    }
}

} // namespace mslam
