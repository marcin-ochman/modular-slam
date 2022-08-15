#define CATCH_CONFIG_MAIN

#include "modular_slam/plugin_loader.hpp"
#include <catch2/catch.hpp>
#include <filesystem>

using std::filesystem::path;

namespace mslam
{

SCENARIO("Loading plugins")
{
    GIVEN("DLL library with factory method")
    {
        const path dummyLibDir{DUMMY_DIR};
        const auto libPath = dummyLibDir / "libdummy.so";

        auto factoryFunction = loadFactoryMethod<int>(libPath.string(), "dummyPluginFactory");
        WHEN("Loading library")
        {
            REQUIRE(!factoryFunction.empty());
            REQUIRE(factoryFunction() == nullptr);
        }
    }
}
} // namespace mslam
