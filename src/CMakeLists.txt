add_executable(loop_detection_test ut/loop_detection_test.cpp ut/plugin_loader_test.cpp)
target_link_libraries(loop_detection_test PRIVATE Catch2::Catch2 modular_slam boost::boost dl)
ParseAndAddCatchTests(loop_detection_test)
