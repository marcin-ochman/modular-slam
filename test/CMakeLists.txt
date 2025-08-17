cmake_minimum_required(VERSION 3.19)
project(ModularSlamTests)

enable_testing()

find_package(Catch2 3 REQUIRED)
find_package(trompeloeil REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED)

include(Catch)

if(NOT TARGET ModularSlam::ModularSlam)
  find_package(ModularSlam CONFIG REQUIRED)
endif()

add_library(dummy SHARED dummy.cpp)
target_link_libraries(dummy PRIVATE Boost::boost)
add_executable(
  modular_slam_tests plugin_loader_test.cpp rgbd_file_provider_test.cpp
                     parameter_register_test.cpp slam_builder_test.cpp)
target_compile_definitions(
  modular_slam_tests PRIVATE DUMMY_DIR="$<TARGET_FILE_DIR:dummy>"
                             TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_link_libraries(
  modular_slam_tests
  PRIVATE ModularSlam::ModularSlam Catch2::Catch2WithMain
          trompeloeil::trompeloeil ${OpenCV_LIBS} Boost::filesystem)

catch_discover_tests(modular_slam_tests)
