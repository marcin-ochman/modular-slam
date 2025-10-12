cmake_minimum_required(VERSION 3.21)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF)

project(
  modular-slam
  VERSION 1.0.0
  DESCRIPTION "Algorithms and tools for building VSLAM systems"
  HOMEPAGE_URL "%%myurl%%"
  LANGUAGES CXX C)

add_subdirectory(src/app/cpp)
add_subdirectory(src/lib/modular_slam)
