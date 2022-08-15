cmake_minimum_required(VERSION 3.19)

project(ModularSlam VERSION 0.0.1)

list(
  APPEND CMAKE_MODULE_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(GEN_DOCS "Generate documentation" OFF)

set(CXX_GNU_COMPILE_FLAGS "-Wall -pedantic -Wextra")
set(CXX_CLANG_COMPILE_FLAGS "-Wall -pedantic -Wextra")

add_subdirectory(src)
add_subdirectory(doc)

enable_testing()
add_subdirectory(test)
