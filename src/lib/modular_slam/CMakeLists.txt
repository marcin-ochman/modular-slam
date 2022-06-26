cmake_minimum_required(VERSION 3.19)
project(ModularSlam VERSION 0.0.1)

find_package(Threads)
find_package(Boost 1.74.0 REQUIRED COMPONENTS program_options log system filesystem)
find_package(Catch2 2.11.0 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(nlohmann_json 3.10.5 REQUIRED)
find_package(Qt6 COMPONENTS Widgets)
find_package(Ceres REQUIRED)
find_package(trompeloeil)
find_package(Doxygen)

add_library(
  ModularSlam
  realsense_camera.cpp
  rgbd_file_provider.cpp
  basic_parameters_handler.cpp
  parameters_handler.cpp
)

add_library(ModularSlam::ModularSlam ALIAS ModularSlam)

target_include_directories(
  ModularSlam
  PUBLIC
  $<BUILD_INTERFACE:${ModularSlam_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${ModularSlam_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  ModularSlam PRIVATE ${realsense2_LIBRARY}
  opencv_core
  opencv_video
  opencv_imgproc
  opencv_highgui
  Ceres::ceres
  PUBLIC
  Boost::filesystem
)

install(
  TARGETS ModularSlam EXPORT ModularSlamTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/
)

install(
  EXPORT ModularSlamTargets
  DESTINATION lib/cmake/ModularSlam
  FILE ModularSlamTargets.cmake
  NAMESPACE ModularSlam::
)

install(
  DIRECTORY include/modular_slam
  DESTINATION include
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(ModularSlamConfigVersion.cmake COMPATIBILITY SameMajorVersion)
install(
  FILES ModularSlamConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/ModularSlamConfigVersion.cmake
  DESTINATION lib/cmake/ModularSlam
)
