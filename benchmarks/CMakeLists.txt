cmake_minimum_required(VERSION 3.16)
project(robot_benchmarks LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")
  message(FATAL_ERROR "Benchmarki wymagają CMAKE_BUILD_TYPE=Release")
endif()

add_executable(technical_cpp examples/cpp_cycle.cpp)
target_compile_features(technical_cpp PRIVATE cxx_std_17)
target_compile_options(technical_cpp PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(technical_cpp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(harness_check EXCLUDE_FROM_ALL test_harness.cpp)
target_compile_features(harness_check PRIVATE cxx_std_17)
target_compile_options(harness_check PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(harness_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB MOTION_CAN_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/../ros2_ws/src/mks_servo_can/src/*.cpp")
add_executable(motion_bus motion_bus.cpp ${MOTION_CAN_SOURCES})
target_compile_features(motion_bus PRIVATE cxx_std_17)
target_include_directories(motion_bus PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../ros2_ws/src/mks_servo_can/include)
target_link_libraries(motion_bus PRIVATE pthread)
