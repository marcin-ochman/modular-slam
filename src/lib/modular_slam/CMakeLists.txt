add_library(modular_slam modular_slam.cpp)
target_include_directories(modular_slam PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                               "$<INSTALL_INTERFACE:include>")

add_library(modular_slam::modular_slam ALIAS modular_slam)

install(
  TARGETS modular_slam
  EXPORT modular_slam-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION include)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/modular_slamConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modular_slam_config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/modular_slamConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_DATADIR}/modular_slam/cmake")

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/modular_slamConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/modular_slamConfigVersion.cmake"
        DESTINATION "${CMAKE_INSTALL_DATADIR}/modular_slam/cmake")

install(
  EXPORT modular_slam-targets
  FILE modular_slam-targets.cmake
  NAMESPACE modular_slam::
  DESTINATION "${CMAKE_INSTALL_DATADIR}/modular_slam/cmake")
