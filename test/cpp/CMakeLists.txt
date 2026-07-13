cmake_minimum_required(VERSION 3.23)

project(ModularSlamTests LANGUAGES CXX)

enable_testing()

include(CTest)

if(NOT TARGET modular_slam::modular_slam)
  find_package(modular_slam CONFIG REQUIRED)
endif()

find_package(Catch2 3 REQUIRED)
include(Catch)

add_executable(tests test_modular_slam.cpp test_time_stamp.cpp test_channels.cpp)
target_link_libraries(tests PRIVATE modular_slam::modular_slam Catch2::Catch2WithMain)

catch_discover_tests(
  tests
  TEST_PREFIX
  "unittests."
  REPORTER
  XML
  OUTPUT_DIR
  .
  OUTPUT_PREFIX
  "unittests."
  OUTPUT_SUFFIX
  .xml)
