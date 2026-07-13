#include "modular_slam/modular_slam.hpp"
#include "modular_slam/slam/slam_builder.hpp"

namespace msl = mslam::log;

int main(int /*argc*/, char* /*argv*/[])
{
    msl::info("Welcome to modular slam!");

    auto builder = mslam::SlamBuilder{};

    return 0;
}
