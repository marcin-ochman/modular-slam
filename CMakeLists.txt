cmake_minimum_required(VERSION 3.28)

project(
  modular-slam
  VERSION 1.0.0
  DESCRIPTION "Algorithms and tools for building VSLAM systems"
  HOMEPAGE_URL "%%myurl%%"
  LANGUAGES CXX C)

include(cmake/modular_slam.cmake)

option(MODULAR_SLAM_WARNINGS_AS_ERRORS "Warnings are treated as errors. Useful in development." OFF)
option(MODULAR_SLAM_ENABLE_CLANG_TIDY "Enables static analysis with clang-tidy during build" ON)
option(MODULAR_SLAM_COVERAGE_ENABLED "Builds library and tests with coverage enabled." OFF)
cmake_dependent_option(
  MODULAR_SLAM_BUILD_APPS
  "Builds applications"
  ON
  "NOT MODULAR_SLAM_COVERAGE_ENABLED"
  OFF)
option(MODULAR_SLAM_ENABLE_SANITIZER_ADDRESS "Enable address sanitizer" OFF)
option(MODULAR_SLAM_ENABLE_SANITIZER_LEAK "Enable leak sanitizer" OFF)
option(MODULAR_SLAM_ENABLE_SANITIZER_UNDEFINED "Enable undefined sanitizer" OFF)
option(MODULAR_SLAM_ENABLE_SANITIZER_THREAD "Enable thread sanitizer" OFF)
option(MODULAR_SLAM_ENABLE_SANITIZER_MEMORY "Enable memory sanitizer" OFF)

init_modular_slam_project()

add_subdirectory(src/lib/modular_slam)

if(MODULAR_SLAM_BUILD_APPS)
  add_subdirectory(src/app/cpp)
endif()

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()
