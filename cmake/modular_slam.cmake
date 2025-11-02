include_guard(GLOBAL)

include(CPack)
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)
include(CMakeDependentOption)

include(${CMAKE_CURRENT_LIST_DIR}/static_analyzers.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/sanitizers.cmake)

macro(init_modular_slam_project)

  set(CMAKE_CXX_STANDARD 23)
  set(CMAKE_CXX_EXTENSIONS OFF)

  if(MODULAR_SLAM_ENABLE_CLANG_TIDY)
    find_program(CLANGTIDY clang-tidy)

    if(NOT CLANGTIDY)
      message(AUTHOR_WARNING "clang-tidy requested but executable not found")
    endif()
  endif()

  add_library(modular_slam_options INTERFACE)
  add_library(modular_slam::modular_slam_options ALIAS modular_slam_options)

  modular_slam_enable_sanitizers(
    modular_slam_options
    ${MODULAR_SLAM_ENABLE_SANITIZER_ADDRESS}
    ${MODULAR_SLAM_ENABLE_SANITIZER_LEAK}
    ${MODULAR_SLAM_ENABLE_SANITIZER_UNDEFINED}
    ${MODULAR_SLAM_ENABLE_SANITIZER_THREAD}
    ${MODULAR_SLAM_ENABLE_SANITIZER_MEMORY})
endmacro()

function(modular_slam_add_library target)
  set(options
      STATIC
      SHARED
      MODULE
      OBJECT
      INTERFACE
      EXCLUDE_FROM_ALL
      NO_TIDY)
  set(oneValueArgs "")
  set(multiValueArgs SOURCES)

  cmake_parse_arguments(
    MSAL # MSAL - Modular SLAM Add Library
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN})

  if(MSAL_INTERFACE)
    set(_type INTERFACE)
  elseif(MSAL_OBJECT)
    set(_type OBJECT)
  elseif(MSAL_MODULE)
    set(_type MODULE)
  elseif(MSAL_SHARED)
    set(_type SHARED)
  else()
    set(_type STATIC)
  endif()

  if(MSAL_EXCLUDE_FROM_ALL)
    add_library(${target} EXCLUDE_FROM_ALL ${_type} ${MSAL_SOURCES})
  else()
    add_library(${target} ${_type} ${MSAL_SOURCES})
  endif()

  if(NOT MSAL_INTERFACE)
    target_link_libraries(
      ${target}
      PUBLIC ${MSAL_PUBLIC_LIBS}
      PRIVATE ${MSAL_PRIVATE_LIBS})

    if(MODULAR_SLAM_ENABLE_CLANG_TIDY AND NOT MSAL_NO_TIDY)
      modular_slam_enable_clang_tidy(${target} ${MODULAR_SLAM_WARNINGS_AS_ERRORS})
    endif()
  endif()
endfunction()

function(modular_slam_add_executable target)
  set(options EXCLUDE_FROM_ALL NO_TIDY)
  set(oneValueArgs "")
  set(multiValueArgs SOURCES PUBLIC_LIBS PRIVATE_LIBS)
  cmake_parse_arguments(
    MSAE
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN})

  if(MSAE_EXCLUDE_FROM_ALL)
    add_executable(${target} EXCLUDE_FROM_ALL ${MSAE_SOURCES})
  else()
    add_executable(${target} ${MSAE_SOURCES})
  endif()

  target_link_libraries(${target} PUBLIC modular_slam::modular_slam_options)

  if(MODULAR_SLAM_ENABLE_CLANG_TIDY AND NOT MSAE_NO_TIDY)
    modular_slam_enable_clang_tidy(${target} ${MODULAR_SLAM_WARNINGS_AS_ERRORS})
  endif()
endfunction()
