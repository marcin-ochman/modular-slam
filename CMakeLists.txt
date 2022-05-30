cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME "modular-slam")
project(${PROJECT_NAME})

list(
  APPEND CMAKE_MODULE_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake
)

set(${PROJECT_NAME}_VERSION_MAJOR 0)
set(${PROJECT_NAME}_VERSION_MINOR 1)
set(${PROJECT_NAME}_VERSION_PATCH 0)
set(${PROJECT_NAME}_VERSION "${PROJECT_NAME_VERSION_MAJOR}.${PROJECT_NAME_VERSION_MINOR}.${PROJECT_NAME_VERSION_PATCH}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(GEN_DOCS "Generate documentation" OFF)

set(CXX_GNU_COMPILE_FLAGS "-Wall -pedantic -Wextra ")
set(CXX_CLANG_COMPILE_FLAGS "-Wall -pedantic -Wextra")


if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  list(APPEND CMAKE_CXX_FLAGS ${CXX_GNU_COMPILE_FLAGS})
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  list(APPEND CMAKE_CXX_FLAGS ${CXX_CLANG_COMPILE_FLAGS})
endif()

include(CTest)
include(ParseAndAddCatchTests)

find_package(Threads)
find_package(Boost 1.74.0 REQUIRED COMPONENTS program_options log system filesystem)
find_package(Catch2 2.11.0 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(nlohmann_json 3.10.5 REQUIRED)
find_package(Qt5 COMPONENTS Widgets)
find_package(Ceres REQUIRED)

find_package(Doxygen)

add_subdirectory(doc)


add_library(modular_slam src/realsense_camera.cpp src/rgbd_file_provider.cpp)
add_library(dummy SHARED src/ut/dummy.cpp)

target_include_directories(modular_slam PUBLIC include)
target_link_libraries(
  modular_slam PRIVATE ${realsense2_LIBRARY}
  opencv_core
  opencv_video
  opencv_imgproc
  opencv_highgui
  Ceres::ceres
)

add_executable(modular_slam_tests src/ut/plugin_loader_test.cpp src/ut/rgbd_file_provider_test.cpp)
target_compile_definitions(modular_slam_tests PRIVATE DUMMY_DIR="$<TARGET_FILE_DIR:dummy>" TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_link_libraries(modular_slam_tests PRIVATE Catch2::Catch2 modular_slam Boost::boost Boost::filesystem dl opencv_core)

# add_executable(data_provider_test)
# target_compile_definitions(data_provider_test)
# target_link_libraries(
#   data_provider_test PRIVATE Catch2::Catch2 modular_slam Boost::boost
#   opencv_core
# )

parseandaddcatchtests(modular_slam_tests)

add_subdirectory(src/app)
