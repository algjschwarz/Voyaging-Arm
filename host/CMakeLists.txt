cmake_minimum_required(VERSION 3.20)
project(arm_host LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(arm_core src/kinematics.cpp)
target_include_directories(arm_core PUBLIC include)

add_executable(arm_app src/main.cpp)
target_link_libraries(arm_app PRIVATE arm_core)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)