cmake_minimum_required(VERSION 3.19)
project(ModularSlamApplications VERSION 0.0.1)

find_package(Qt6 COMPONENTS Widgets OpenGL OpenGLWidgets Gui REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glm REQUIRED)

if(NOT TARGET ModularSlam::ModularSlam)
  find_package(ModularSlam CONFIG REQUIRED)
endif()

qt_wrap_ui(
  MSLAM_VIEWER_UIS
  viewer/ui/viewer_main_window.ui
  viewer/ui/image_viewer.ui
)

qt_wrap_cpp(
  MSLAM_VIEWER_HEADERS
  viewer/image_viewer.hpp
  viewer/slam_thread.hpp
  viewer/pointcloud_viewer.hpp
)

qt_add_executable(
  Viewer
  ${MSLAM_VIEWER_UIS}
  ${MSLAM_VIEWER_HEADERS}
  viewer/viewer.cpp
  viewer/viewer_main_window.cpp
  viewer/image_viewer.cpp
  viewer/pointcloud_viewer.cpp
)

add_executable(ModularSlam::Viewer ALIAS Viewer)
target_include_directories(Viewer PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/viewer)

target_link_libraries(
  Viewer PRIVATE
  Qt6::Widgets
  Qt6::OpenGL
  Qt6::OpenGLWidgets
  Qt6::Gui
  ModularSlam::ModularSlam
  OpenGL::GLU
  glm
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
