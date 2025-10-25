add_executable(run_slam run_slam.cpp)

target_link_libraries(run_slam modular_slam::modular_slam)

install(TARGETS run_slam RUNTIME DESTINATION bin)
