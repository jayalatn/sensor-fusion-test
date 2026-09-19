cmake_minimum_required(VERSION 3.20)
project(sensor_fusion CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
if(MSVC)
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(fusion STATIC
  src/robot.cpp
  src/sensors.cpp
  src/ekf.cpp
  src/scenario.cpp
  src/simulated_robot.cpp)
target_include_directories(fusion PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(fusion_harness apps/harness.cpp)
target_link_libraries(fusion_harness PRIVATE fusion)

find_package(GTest QUIET)
if(NOT GTest_FOUND)
  include(FetchContent)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.2
    GIT_SHALLOW    TRUE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)  # match MSVC's /MD default
  FetchContent_MakeAvailable(googletest)
endif()

add_executable(fusion_tests tests/tests.cpp)
target_link_libraries(fusion_tests PRIVATE fusion GTest::gtest_main)

enable_testing()
include(GoogleTest)
gtest_discover_tests(fusion_tests)
