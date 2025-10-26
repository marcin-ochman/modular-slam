cmake_minimum_required(VERSION 3.28)

include(CPack)
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF)

project(
  modular-slam
  VERSION 1.0.0
  DESCRIPTION "Algorithms and tools for building VSLAM systems"
  HOMEPAGE_URL "%%myurl%%"
  LANGUAGES CXX C)

option(WARNINGS_AS_ERRORS "Warnings are treated as errors. Useful in development." OFF)

include("cmake/static_analyzers.cmake")

modular_slam_enable_clang_tidy(${WARNINGS_AS_ERRORS})

add_subdirectory(src/app/cpp)
add_subdirectory(src/lib/modular_slam)

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()
