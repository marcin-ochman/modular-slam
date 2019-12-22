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
include(Conan)
include(ParseAndAddCatchTests)

conan_cmake_run(
  CONANFILE conanfile.txt
  BASIC_SETUP
  BUILD missing)

include(${CMAKE_BINARY_DIR}/conan_paths.cmake)

find_package(Threads)
find_package(boost 1.66.0 REQUIRED COMPONENTS program_options log)
find_package(catch2  2.2.2  REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(trompeloeil REQUIRED)
find_package(jsonformoderncpp 3.7.0 REQUIRED)
find_package(Doxygen)

add_subdirectory(doc)
add_subdirectory(src)
