cmake_minimum_required(VERSION 3.28)

project(
  modular-slam
  VERSION 1.0.0
  DESCRIPTION "Algorithms and tools for building VSLAM systems"
  HOMEPAGE_URL "%%myurl%%"
  LANGUAGES CXX C)

include(cmake/modular_slam.cmake)

option(MODULAR_SLAM_WARNINGS_AS_ERRORS "Warnings are treated as errors. Useful in development." OFF)
option(MODULAR_SLAM_ENABLE_CLANG_TIDY "Enables static analysis with clang-tidy during build" OFF)
option(MODULAR_SLAM_ENABLE_COVERAGE "Builds library and tests with coverage enabled." OFF)
option(MODULAR_SLAM_BUILD_PYTHON_BINDINGS "" OFF)
cmake_dependent_option(
  MODULAR_SLAM_BUILD_APPS
  "Builds applications"
  ON
  "NOT MODULAR_SLAM_ENABLE_COVERAGE"
  OFF)
option(MODULAR_SLAM_ENABLE_SANITIZER_ADDRESS "Enable address sanitizer" OFF)
option(MODULAR_SLAM_ENABLE_SANITIZER_UNDEFINED "Enable undefined sanitizer" OFF)
option(MODULAR_SLAM_ENABLE_SANITIZER_THREAD "Enable thread sanitizer" OFF)

init_modular_slam_project()

add_subdirectory(src/cpp/lib/modular_slam)

if(MODULAR_SLAM_BUILD_APPS)
  add_subdirectory(src/cpp/app/cpp)
endif()

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()
