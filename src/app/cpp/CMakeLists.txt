modular_slam_add_executable(run_slam SOURCES run_slam.cpp)

target_link_libraries(
  run_slam
  PRIVATE modular_slam::modular_slam
  PUBLIC modular_slam::modular_slam_options)

install(TARGETS run_slam RUNTIME DESTINATION bin)
