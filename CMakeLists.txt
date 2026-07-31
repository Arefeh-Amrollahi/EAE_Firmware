cmake_minimum_required(VERSION 3.16)
project(eae_firmware VERSION 1.0.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Standards and warnings.
# Warnings are set high deliberately: on a controls target, an uninitialised
# variable is a machine that moves when it should not.
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_library(eae_warnings INTERFACE)
if(MSVC)
  target_compile_options(eae_warnings INTERFACE /W4)
else()
  target_compile_options(eae_warnings INTERFACE
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion)
endif()

# ---------------------------------------------------------------------------
# Core library.  Kept separate from the executable so the tests link the same
# objects the application does, rather than a recompiled copy.
# ---------------------------------------------------------------------------
add_library(eae_core
  src/can_bus.cpp
  src/pid.cpp
  src/state_machine.cpp
  src/cooling_controller.cpp
  src/plant.cpp
)
target_include_directories(eae_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(eae_core PRIVATE eae_warnings)

add_executable(eae_cooling src/main.cpp)
target_link_libraries(eae_cooling PRIVATE eae_core eae_warnings)

# ---------------------------------------------------------------------------
# Tests.  GoogleTest is fetched at configure time rather than vendored into the
# repository, per the brief: no dependencies are shipped in the project tree.
# Set -DEAE_BUILD_TESTS=OFF for an offline or cross-compiled build.
# ---------------------------------------------------------------------------
option(EAE_BUILD_TESTS "Build unit tests" ON)

if(EAE_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.2
    GIT_SHALLOW    TRUE
  )
  # Pinned to a release tag, not a moving branch: a build that changes
  # behaviour because an upstream default branch moved is not reproducible.
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  add_executable(eae_tests tests/test_all.cpp)
  target_link_libraries(eae_tests PRIVATE eae_core GTest::gtest_main)

  include(GoogleTest)
  gtest_discover_tests(eae_tests)
endif()
