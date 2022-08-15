#include "modular_slam/basic_parameters_handler.hpp"
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
    MAKE_MOCK2(registerParameter, bool(const ParameterDefinition& paramDefinition, const ParameterValue& value),
               override);
    MAKE_MOCK2(setParameter, bool(const std::string& name, const ParameterValue& value), override);
    MAKE_CONST_MOCK1(getParameter, std::optional<ParameterValue>(const std::string& name), override);
};

SCENARIO("Testing parameters handling")
{
    GIVEN("Basic")
    {
        BasicParameterHandler parameterHandler;
        WHEN("Parameter is not registered")
        {
            THEN("Setting parameter is not valid and returns false")
            {
                REQUIRE_FALSE(parameterHandler.setParameter("/my/parameter", 0));
            }

            THEN("Getting parameter is not valid")
            {
                auto value = parameterHandler.getParameter("my/parameter");
                REQUIRE_FALSE(value.has_value());
            }
        }

        WHEN("Parameter is registered")
        {
            ParameterDefinition definition = makeChoiceParameter("my/parameter", {0, 1, 2});
            parameterHandler.registerParameter(definition, 0);

            THEN("Setting parameter is valid when choice is ok")
            {
                REQUIRE(parameterHandler.setParameter("my/parameter", 0));
                REQUIRE_FALSE(parameterHandler.setParameter("my/parameter", 5));
            }

            THEN("Getting parameter is valid")
            {
                auto value = parameterHandler.getParameter("my/parameter");
                REQUIRE(value.has_value());
                REQUIRE(std::get<0>(value.value()) == 0);
            }
        }
    }
}

} // namespace mslam
