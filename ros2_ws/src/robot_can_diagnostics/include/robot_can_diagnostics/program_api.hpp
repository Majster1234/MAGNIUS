#pragma once
#include "mks_servo_can/bus_runtime.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_control_interfaces/msg/motion_settings.hpp"
#include "robot_control_interfaces/msg/motion_program_status.hpp"
#include "robot_control_interfaces/srv/prepare_motion_program.hpp"
#include "robot_control_interfaces/srv/get_motion_program.hpp"
#include "robot_control_interfaces/action/execute_motion_program.hpp"
namespace robot_can_diagnostics {
class ProgramApi {
 public:
  ProgramApi(rclcpp::Node *node,std::shared_ptr<mks_servo_can::BusRuntime> runtime);
 private:
  using Action=robot_control_interfaces::action::ExecuteMotionProgram;
  using Handle=rclcpp_action::ServerGoalHandle<Action>;
  std::shared_ptr<mks_servo_can::BusRuntime> runtime_;
  rclcpp::Service<robot_control_interfaces::srv::PrepareMotionProgram>::SharedPtr prepare_;
  rclcpp::Service<robot_control_interfaces::srv::GetMotionProgram>::SharedPtr status_;
  rclcpp::Subscription<robot_control_interfaces::msg::MotionSettings>::SharedPtr settings_;
  rclcpp_action::Server<Action>::SharedPtr action_;
  std::shared_ptr<Handle> handle_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr model_;
};
}
