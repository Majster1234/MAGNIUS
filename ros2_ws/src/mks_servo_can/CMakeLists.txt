cmake_minimum_required(VERSION 3.8)
project(mks_servo_can)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/protocol.cpp
  src/socketcan.cpp
  src/trace.cpp
  src/bus_manager.cpp
  src/configuration.cpp
  src/motion_executor.cpp
  src/program_bus.cpp
  src/motion_sample.cpp
  src/streaming_motion.cpp
  src/tcp_jog.cpp
  src/manual_motor_controller.cpp
  src/bus_runtime.cpp)
target_link_libraries(${PROJECT_NAME} Threads::Threads)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Threads)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol ${PROJECT_NAME})
  ament_add_gtest(test_bus_manager test/test_bus_manager.cpp)
  target_link_libraries(test_bus_manager ${PROJECT_NAME})
  ament_add_gtest(test_manual_motor_controller test/test_manual_motor_controller.cpp)
  target_link_libraries(test_manual_motor_controller ${PROJECT_NAME})
  ament_add_gtest(test_trace test/test_trace.cpp)
  target_link_libraries(test_trace ${PROJECT_NAME})
  ament_add_gtest(test_shaft_trace test/test_shaft_trace.cpp)
  target_link_libraries(test_shaft_trace ${PROJECT_NAME})
  ament_add_gtest(test_motion_executor test/test_motion_executor.cpp)
  target_link_libraries(test_motion_executor ${PROJECT_NAME})
  ament_add_gtest(test_configuration_revision test/test_configuration_revision.cpp)
  target_link_libraries(test_configuration_revision ${PROJECT_NAME})
  ament_add_gtest(test_configuration_continuity test/test_configuration_continuity.cpp)
  target_link_libraries(test_configuration_continuity ${PROJECT_NAME})
  ament_add_gtest(test_motion_sample test/test_motion_sample.cpp)
  target_link_libraries(test_motion_sample ${PROJECT_NAME})
  ament_add_gtest(test_streaming_motion test/test_streaming_motion.cpp)
  target_link_libraries(test_streaming_motion ${PROJECT_NAME})
  ament_add_gtest(test_socketcan_async test/test_socketcan_async.cpp)
  target_link_libraries(test_socketcan_async ${PROJECT_NAME})
  ament_add_gtest(test_tcp_jog test/test_tcp_jog.cpp)
  target_link_libraries(test_tcp_jog ${PROJECT_NAME})
  ament_add_gtest(test_enable_all test/test_enable_all.cpp)
  target_link_libraries(test_enable_all ${PROJECT_NAME})
  ament_add_gtest(test_idle_enable test/test_idle_enable.cpp)
  target_link_libraries(test_idle_enable ${PROJECT_NAME})
endif()

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
