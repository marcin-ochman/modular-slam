find_program(CLANG_TIDY_EXE NAMES clang-tidy)

if(CLANG_TIDY_EXE AND STATIC_CODE_ANALYSIS)
  message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
  set(CLANG_TIDY_COMMAND "${CLANG_TIDY_EXE}"
                         "--config-file=${PROJECT_SOURCE_DIR}/.clang-tidy")

  set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_COMMAND}")

endif()

add_subdirectory(lib)
add_subdirectory(app)
