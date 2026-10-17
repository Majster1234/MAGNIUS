cmake_minimum_required(VERSION 3.8)
project(mks_servo_can)

find_package(ament_cmake REQUIRED)

# Add library targets, header installation and exports with the implementation.
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
