add_executable(dummy_test ut/dummy.cpp)

target_link_libraries(dummy_test PRIVATE catch2::catch2)

ParseAndAddCatchTests(dummy_test)
