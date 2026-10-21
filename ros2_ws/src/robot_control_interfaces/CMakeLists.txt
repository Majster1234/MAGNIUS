cmake_minimum_required(VERSION 3.8)
project(robot_control_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ManualAxisState.msg" "msg/ManualState.msg"
  "msg/JointConfiguration.msg" "msg/RobotConfiguration.msg"
  "srv/SetMotorEnable.srv" "srv/ZeroMotor.srv" "srv/JogMotor.srv"
  "msg/PlanningContext.msg" "msg/MksPlanSample.msg" "msg/CartesianPlan.msg"
  "srv/SolveIK.srv" "action/ComputeCartesianPath.action"
  "action/ComputePreviewMotion.action"
  "srv/JogJoint.srv" "srv/ConfigureRobot.srv" "srv/ConfigurationCommand.srv"
  "msg/MotionSettings.msg" "msg/MotionProgramStatus.msg"
  "srv/PrepareMotionProgram.srv" "srv/GetMotionProgram.srv" "action/ExecuteMotionProgram.action" "action/ComputePhysicalPtp.action"
  DEPENDENCIES geometry_msgs trajectory_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
