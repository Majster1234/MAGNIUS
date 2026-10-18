cmake_minimum_required(VERSION 3.8)
project(robot_control_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ManualAxisState.msg" "msg/ManualState.msg"
  "msg/JointConfiguration.msg" "msg/RobotConfiguration.msg"
  "srv/SetMotorEnable.srv" "srv/ZeroMotor.srv" "srv/JogMotor.srv"
  "srv/JogJoint.srv" "srv/ConfigureRobot.srv" "srv/ConfigurationCommand.srv")
ament_export_dependencies(rosidl_default_runtime)
ament_package()
