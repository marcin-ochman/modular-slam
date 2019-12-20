#define CATCH_CONFIG_MAIN

#include <catch.hpp>


SCENARIO("Dummy Scenario")
{
  GIVEN("Dummy given")
    {
      WHEN("Dummy when")
        {
          THEN("Dummy then")
            {
              REQUIRE(true);
            }
        }
    }
}
