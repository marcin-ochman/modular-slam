cmake_minimum_required(VERSION 3.19)
project(ModularSlamApplications VERSION 0.0.1)

find_package(Qt6 COMPONENTS Widgets OpenGL OpenGLWidgets Gui Charts REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glm REQUIRED)
find_package(OpenCV 4 REQUIRED)

if(NOT TARGET ModularSlam::ModularSlam)
  find_package(ModularSlam CONFIG REQUIRED)
endif()

qt_wrap_ui(
  MSLAM_VIEWER_UIS
  viewer/ui/viewer_main_window.ui
  viewer/ui/image_viewer.ui
  viewer/ui/depth_image_viewer.ui
  viewer/ui/slam_statistics.ui
)

qt_wrap_cpp(
  MSLAM_VIEWER_HEADERS
  viewer/image_viewer.hpp
  viewer/depth_image_viewer.hpp
  viewer/slam_thread.hpp
  viewer/pointcloud_viewer.hpp
)

qt_add_resources(MSLAM_VIEWER_QRC viewer/qrc/viewer.qrc)

qt_add_executable(
  Viewer
  ${MSLAM_VIEWER_UIS}
  ${MSLAM_VIEWER_HEADERS}
  ${MSLAM_VIEWER_QRC}
  viewer/colorbar.cpp
  viewer/depth_image_viewer.cpp
  viewer/grid.cpp
  viewer/image_viewer.cpp
  viewer/pointcloud_viewer.cpp
  viewer/viewer.cpp
  viewer/viewer_main_window.cpp
  viewer/slam_statistics_widget.cpp
  viewer/slam_thread.cpp
  viewer/map_2d_widget.cpp
  viewer/keyframe_thumbnail.cpp
)

add_executable(ModularSlam::Viewer ALIAS Viewer)
target_include_directories(Viewer PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/viewer)

target_link_libraries(
  Viewer PRIVATE
  Qt6::Widgets
  Qt6::OpenGL
  Qt6::OpenGLWidgets
  Qt6::Gui
  Qt6::Charts
  ModularSlam::ModularSlam
  OpenGL::GLU
  glm
  opencv_core
  opencv_imgproc
)

add_executable(RgbdSlam slam/rgbd_slam.cpp)
target_link_libraries(
  RgbdSlam PRIVATE ModularSlam::ModularSlam
  Qt6::Widgets
  Qt6::OpenGL
  Qt6::OpenGLWidgets
  Qt6::Gui
  Qt6::Charts

  ModularSlam::ModularSlam
  OpenGL::GLU
  glm
  opencv_core
  opencv_imgproc
)

install(
  TARGETS Viewer EXPORT ModularSlamApplicationsTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/
)

install(
  EXPORT ModularSlamApplicationsTargets
  DESTINATION lib/cmake/ModularSlam
  FILE ModularSlamApplicationsTargets.cmake
  NAMESPACE ModularSlam::
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(ModularSlamApplicationsConfigVersion.cmake COMPATIBILITY SameMajorVersion)
install(
  FILES ModularSlamApplicationsConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/ModularSlamConfigVersion.cmake
  DESTINATION lib/cmake/ModularSlam
)
