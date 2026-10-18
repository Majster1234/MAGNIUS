#pragma once
#include <future>
#include "mks_servo_can/bus_runtime.hpp"
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"
#include "robot_control_interfaces/msg/manual_state.hpp"
#include "robot_control_interfaces/srv/set_motor_enable.hpp"
#include "robot_control_interfaces/srv/zero_motor.hpp"
#include "robot_control_interfaces/srv/jog_motor.hpp"
#include "robot_control_interfaces/srv/jog_joint.hpp"
#include "robot_control_interfaces/srv/configure_robot.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace robot_can_diagnostics {
class DiagnosticsNode : public rclcpp::Node {
 public:
  DiagnosticsNode(std::shared_ptr<mks_servo_can::BusRuntime> runtime,
                  const mks_servo_can::BusConfig &config);
 private:
  void publish();
  void publish_manual();
  std::shared_ptr<mks_servo_can::BusRuntime> runtime_;
  mks_servo_can::BusConfig config_;
  bool resetting_{false};
  mks_servo_can::Time last_manual_publish_{};
  std::string reset_error_;
  std::future<int> reset_future_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr scan_, reset_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_;
  rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr lease_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr manual_timer_;
  rclcpp::Publisher<robot_control_interfaces::msg::ManualState>::SharedPtr manual_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr preview_publisher_;
  std::array<mks_servo_can::Time, mks_servo_can::kAxes> last_preview_{};
  rclcpp::Service<robot_control_interfaces::srv::SetMotorEnable>::SharedPtr enable_;
  rclcpp::Service<robot_control_interfaces::srv::ZeroMotor>::SharedPtr zero_;
  rclcpp::Service<robot_control_interfaces::srv::JogMotor>::SharedPtr jog_;
  rclcpp::Service<robot_control_interfaces::srv::JogJoint>::SharedPtr joint_jog_;
  rclcpp::Service<robot_control_interfaces::srv::ConfigureRobot>::SharedPtr configuration_;
};
}  // namespace robot_can_diagnostics
