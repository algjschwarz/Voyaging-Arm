cmake_minimum_required(VERSION 3.20)
project(arm_host LANGUAGES CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

add_library(arm_core src/kinematics.cpp src/limits.cpp src/motion.cpp src/servo_map.cpp src/serial_link.cpp)
target_include_directories(arm_core PUBLIC include)

# Servo calibration, written by arm_calibrate and read by arm.
target_compile_definitions(arm_core PUBLIC ARM_CALIBRATION_FILE="${CMAKE_CURRENT_SOURCE_DIR}/calibration/servos.txt")

add_executable(arm_calibrate src/calibrate.cpp)
target_link_libraries(arm_calibrate PRIVATE arm_core)

include(FetchContent)
FetchContent_Declare(googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip)
FetchContent_MakeAvailable(googletest)

enable_testing()
add_executable(arm_tests tests/test_kinematics.cpp tests/test_limits.cpp tests/test_motion.cpp tests/test_servo_map.cpp)
target_link_libraries(arm_tests PRIVATE arm_core GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(arm_tests)

FetchContent_Declare(raylib
  URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.zip)
FetchContent_MakeAvailable(raylib)

# raygui is header-only; we only need its src dir on the include path.
FetchContent_Declare(raygui
  URL https://github.com/raysan5/raygui/archive/refs/tags/4.0.zip)
FetchContent_MakeAvailable(raygui)

# The simulation window, shared by the simulation-only program and the real-arm program.
add_library(arm_simulation_lib src/simulation.cpp)
target_include_directories(arm_simulation_lib SYSTEM PRIVATE ${raygui_SOURCE_DIR}/src)
target_link_libraries(arm_simulation_lib PUBLIC arm_core PRIVATE raylib)

add_executable(arm_simulation src/simulation_main.cpp)  # simulation only
target_link_libraries(arm_simulation PRIVATE arm_simulation_lib)

add_executable(arm src/main.cpp)                        # drives the real arm
target_link_libraries(arm PRIVATE arm_simulation_lib)
