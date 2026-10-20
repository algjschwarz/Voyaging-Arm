cmake_minimum_required(VERSION 3.20)
project(arm_host LANGUAGES CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(arm_core src/kinematics.cpp)
target_include_directories(arm_core PUBLIC include)

add_executable(arm src/main.cpp)
target_link_libraries(arm PRIVATE arm_core)

include(FetchContent)
FetchContent_Declare(googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip)
FetchContent_MakeAvailable(googletest)

enable_testing()
add_executable(arm_tests tests/test_kinematics.cpp)
target_link_libraries(arm_tests PRIVATE arm_core GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(arm_tests)