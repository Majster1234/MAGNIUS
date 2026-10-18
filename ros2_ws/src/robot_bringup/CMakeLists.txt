cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/motion_gate.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_motion_gate test/test_motion_gate.py)
  ament_add_pytest_test(test_motion_gate_ros test/test_motion_gate_ros.py TIMEOUT 30)
endif()

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
