#include <catch2/catch_test_macros.hpp>
#include <chrono>

#include "modular_slam/core/time_stamp.hpp"

TEST_CASE("TimeStamp initialization", "[time]")
{
    const auto start =
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock ::now().time_since_epoch())
            .count();
    modular_slam::Timestamp now = modular_slam::Timestamp::now();
    const auto end =
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock ::now().time_since_epoch())
            .count();

    REQUIRE(now.fromEpoch() >= start);
    REQUIRE(now.fromEpoch() <= end);
}
