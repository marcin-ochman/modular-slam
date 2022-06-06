#include "modular_slam/modular_slam.hpp"
#include "modular_slam/parameters_handler.hpp"
#include "modular_slam/rgbd_file_provider.hpp"
#include <catch2/catch.hpp>
#include <catch2/trompeloeil.hpp>

using namespace std::string_literals;

namespace mslam
{

class ParametersHandlerMock : public ParametersHandlerInterface
{
    MAKE_MOCK0(registerParameter, bool(), override);
    MAKE_MOCK2(setParameter, bool(const std::string& name, const std::any value), override);
    MAKE_CONST_MOCK1(getParameter, std::any(const std::string& name), override);
};

SCENARIO("Testing parameters handling")
{
    GIVEN("Instance of SlamComponent and mock of ParamaterHandler")
    {
        auto paramsHandler = std::make_shared<ParametersHandlerMock>();

        THEN("getParameterAs<std::string> calls getParameter function of ParametersHandler")
        {
            mslam::SlamComponent component{paramsHandler};

            REQUIRE_CALL(*paramsHandler, getParameter("/param/name")).RETURN("It's my parameter"s);
            component.getParameterAs<std::string>("/param/name");

            REQUIRE_CALL(*paramsHandler, getParameter("/param/name")).RETURN("It's my parameter"s);
        }
    }
}

} // namespace mslam
