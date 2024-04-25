cmake_minimum_required(VERSION 3.19)
project(ModularSlam VERSION 0.0.1)

find_package(Threads)
find_package(Boost 1.74.0 REQUIRED COMPONENTS program_options log system
                                              filesystem)
find_package(Catch2 3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(nlohmann_json 3.11.2 REQUIRED)
find_package(Ceres REQUIRED)
find_package(spdlog REQUIRED)
find_package(trompeloeil)
find_package(Doxygen)
find_package(DBoW3 REQUIRED)

add_library(
  ModularSlam
  basic_parameters_handler.cpp
  basic_map.cpp
  basic_feature_map_components_factory.cpp
  camera_parameters.cpp
  cv_ransac_pnp.cpp
  ceres_backend.cpp
  ceres_reprojection_error_pnp.cpp
  distributed_cv_feature.cpp
  parameters_handler.cpp
  realsense_camera.cpp
  rgbd_feature_frontend.cpp
  rgbd_file_provider.cpp
  orb_feature.cpp
  orb_relocalizer.cpp
  projection.cpp
  frame.cpp)

add_library(ModularSlam::ModularSlam ALIAS ModularSlam)

target_include_directories(
  ModularSlam
  PUBLIC $<BUILD_INTERFACE:${ModularSlam_BINARY_DIR}/include>
         $<BUILD_INTERFACE:${ModularSlam_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  ModularSlam
  PRIVATE ${OpenCV_LIBS}
          opencv_core
          opencv_video
          opencv_imgproc
          opencv_features2d
          opencv_highgui
          Ceres::ceres
  PUBLIC realsense2::realsense2 Boost::boost Eigen3::Eigen spdlog::spdlog
         dbow3::dbow3)

install(
  TARGETS ModularSlam
  EXPORT ModularSlamTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/)

install(
  EXPORT ModularSlamTargets
  DESTINATION lib/cmake/ModularSlam
  FILE ModularSlamTargets.cmake
  NAMESPACE ModularSlam::)

install(DIRECTORY include/modular_slam DESTINATION include)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(ModularSlamConfigVersion.cmake
                                 COMPATIBILITY SameMajorVersion)
install(FILES cmake/ModularSlamConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/ModularSlamConfigVersion.cmake
        DESTINATION lib/cmake/ModularSlam)
