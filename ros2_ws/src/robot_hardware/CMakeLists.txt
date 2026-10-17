cmake_minimum_required(VERSION 3.8)
project(robot_hardware)

find_package(ament_cmake REQUIRED)

# Add the SystemInterface library and plugin export with the implementation.
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
