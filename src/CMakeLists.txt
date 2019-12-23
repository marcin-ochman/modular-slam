add_executable(dummy_test ut/dummy.cpp)

target_link_libraries(dummy_test PRIVATE Catch2::Catch2)

ParseAndAddCatchTests(dummy_test)
