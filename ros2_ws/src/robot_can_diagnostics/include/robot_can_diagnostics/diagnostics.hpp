#pragma once
#include <atomic>
#include <future>
#include <functional>
#include "robot_can_diagnostics/program_api.hpp"
#include "robot_can_diagnostics/tcp_jog_api.hpp"
#include "mks_servo_can/bus_runtime.hpp"
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"
#include "robot_control_interfaces/msg/manual_state.hpp"
#include "robot_control_interfaces/msg/shaft_trace_batch.hpp"
#include "robot_control_interfaces/srv/set_motor_enable.hpp"
#include "robot_control_interfaces/srv/set_drive_microstep.hpp"
#include "robot_control_interfaces/srv/drive_parameter.hpp"
#include "robot_control_interfaces/srv/set_all_motors_enable.hpp"
#include "robot_control_interfaces/srv/zero_motor.hpp"
#include "robot_control_interfaces/srv/jog_motor.hpp"
#include "robot_control_interfaces/srv/jog_joint.hpp"
#include "robot_control_interfaces/srv/configure_robot.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace robot_can_diagnostics {
class DiagnosticsNode : public rclcpp::Node {
 public:
  DiagnosticsNode(std::shared_ptr<mks_servo_can::BusRuntime> runtime,
                  const mks_servo_can::BusConfig &config, std::function<int()> restart_action = {});
 private:
  void publish();
  void publish_manual();
  void publish_shaft_trace();
  rclcpp::Publisher<robot_control_interfaces::msg::ShaftTraceBatch>::SharedPtr shaft_publisher_;
  rclcpp::TimerBase::SharedPtr shaft_timer_;
  std::shared_ptr<mks_servo_can::BusRuntime> runtime_;
  mks_servo_can::BusConfig config_;
  std::shared_ptr<ProgramApi> program_api_;
  std::shared_ptr<TcpJogApi> tcp_jog_api_;
  // Grupa sterowania: stan ręczny, JOG, STOP i dzierżawy; nie czeka na programy ani diagnostykę.
  rclcpp::CallbackGroup::SharedPtr control_group_;
  std::atomic<bool> resetting_{false};
  mks_servo_can::Time last_manual_publish_{};
  std::string reset_error_;
  std::future<int> reset_future_;
  std::function<int()> restart_action_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr scan_, reset_, drive_parameters_;
  rclcpp::Service<robot_control_interfaces::srv::SetDriveMicrostep>::SharedPtr microstep_;
  rclcpp::Service<robot_control_interfaces::srv::DriveParameter>::SharedPtr drive_parameter_;
  rclcpp::CallbackGroup::SharedPtr drive_parameter_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_;
  rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr lease_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr manual_timer_;
  rclcpp::Publisher<robot_control_interfaces::msg::ManualState>::SharedPtr manual_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr preview_publisher_;
  std::array<mks_servo_can::Time, mks_servo_can::kAxes> last_preview_{};
  rclcpp::Service<robot_control_interfaces::srv::SetMotorEnable>::SharedPtr enable_;
  rclcpp::Service<robot_control_interfaces::srv::SetAllMotorsEnable>::SharedPtr enable_all_;
  rclcpp::Service<robot_control_interfaces::srv::ZeroMotor>::SharedPtr zero_;
  rclcpp::Service<robot_control_interfaces::srv::JogMotor>::SharedPtr jog_;
  rclcpp::Service<robot_control_interfaces::srv::JogJoint>::SharedPtr joint_jog_;
  rclcpp::Service<robot_control_interfaces::srv::ConfigureRobot>::SharedPtr configuration_;
};
}  // namespace robot_can_diagnostics
