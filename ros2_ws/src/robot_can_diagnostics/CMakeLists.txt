cmake_minimum_required(VERSION 3.8)
project(robot_can_diagnostics)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(mks_servo_can REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(can_diagnostics src/can_diagnostics.cpp)
target_compile_features(can_diagnostics PRIVATE cxx_std_17)
target_compile_options(can_diagnostics PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(can_diagnostics diagnostic_msgs mks_servo_can rclcpp std_srvs)
target_link_libraries(can_diagnostics mks_servo_can::mks_servo_can)

install(TARGETS can_diagnostics DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
