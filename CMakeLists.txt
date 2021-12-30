cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME "modular-slam")
project(${PROJECT_NAME})

list(APPEND CMAKE_MODULE_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake
  )

set(${PROJECT_NAME}_VERSION_MAJOR 0)
set(${PROJECT_NAME}_VERSION_MINOR 1)
set(${PROJECT_NAME}_VERSION_PATCH 0)
set(${PROJECT_NAME}_VERSION "${PROJECT_NAME_VERSION_MAJOR}.${PROJECT_NAME_VERSION_MINOR}.${PROJECT_NAME_VERSION_PATCH}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(GEN_DOCS "Generate documentation" OFF)

set(CXX_GNU_COMPILE_FLAGS "-Wall -pedantic -Wextra ")
set(CXX_CLANG_COMPILE_FLAGS "-Wall -pedantic -Wextra")

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  list(APPEND CMAKE_CXX_FLAGS  ${CXX_GNU_COMPILE_FLAGS})
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  list(APPEND CMAKE_CXX_FLAGS  ${CXX_CLANG_COMPILE_FLAGS})
endif()

include(CTest)
include(ParseAndAddCatchTests)

find_package(Threads)
find_package(Boost 1.71.0 REQUIRED COMPONENTS program_options system filesystem)
find_package(Catch2 2.13.0  REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Doxygen)

add_subdirectory(doc)

add_library(modular_slam INTERFACE)
add_library(dummy SHARED src/ut/dummy.cpp)

target_include_directories(modular_slam INTERFACE include)
get_target_property(DUMMY_OUTPUT_DIR dummy BINARY_DIR)

add_executable(plugin_loader_test src/ut/plugin_loader_test.cpp)
target_compile_definitions(plugin_loader_test PRIVATE DUMMY_DIR="${DUMMY_OUTPUT_DIR}")
target_link_libraries(plugin_loader_test PRIVATE Catch2::Catch2 modular_slam Boost::boost Boost::filesystem dl)

ParseAndAddCatchTests(plugin_loader_test)
