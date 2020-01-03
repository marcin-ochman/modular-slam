#include "modular_slam/plugin_loader.hpp"
#include <catch2/catch.hpp>

namespace mslam
{

SCENARIO("Loading plugins")
{
    GIVEN("DLL library with factory method")
    {
        REQUIRE(true);

        WHEN("Loading library") {
          // loadFactoryMethod<int>("", "factory");
        }
    }
}
} // namespace mslam
