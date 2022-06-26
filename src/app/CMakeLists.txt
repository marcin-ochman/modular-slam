cmake_minimum_required(VERSION 3.19)
project(ModularSlamApplications VERSION 0.0.1)

find_package(Qt6 COMPONENTS Widgets REQUIRED)

if(NOT TARGET ModularSlam::ModularSlam)
  find_package(ModularSlam CONFIG REQUIRED)
endif()

qt_wrap_ui(MSLAM_VIEWER_UIS viewer/ui/viewer_main_window.ui)

add_executable(
  Viewer
  ${MSLAM_VIEWER_UIS}
  viewer/viewer.cpp
  viewer/viewer_main_window.cpp
)

add_executable(ModularSlam::Viewer ALIAS Viewer)
target_include_directories(Viewer PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(
  Viewer PRIVATE
  Qt6::Widgets
  ModularSlam::ModularSlam
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
