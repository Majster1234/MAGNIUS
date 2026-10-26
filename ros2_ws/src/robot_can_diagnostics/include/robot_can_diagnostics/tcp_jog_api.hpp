#pragma once
#include "mks_servo_can/bus_runtime.hpp"
#include "robot_kinematics/tcp_jog.hpp"
#include "robot_control_interfaces/srv/jog_tcp.hpp"
#include "robot_control_interfaces/msg/tcp_jog_status.hpp"
#include "robot_control_interfaces/msg/robot_configuration.hpp"
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <mutex>
#include <thread>

namespace robot_can_diagnostics {
class TcpJogApi {
 public:
  TcpJogApi(rclcpp::Node *node,std::shared_ptr<mks_servo_can::BusRuntime> runtime);
  ~TcpJogApi();
 private:
  using Request=robot_control_interfaces::srv::JogTcp::Request;
  using Status=robot_control_interfaces::msg::TcpJogStatus;
  std::string unavailable(mks_servo_can::Time now);
  void command(const Request &request,robot_control_interfaces::srv::JogTcp::Response &response);
  void tick();
  void fail(const std::string &message);
  std::shared_ptr<mks_servo_can::BusRuntime> runtime_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::atomic<bool> running_{true};
  std::thread worker_;
  std::unique_ptr<robot_kinematics::TcpJog> generator_;
  Status status_;
  std::string xml_,model_,revision_,session_,limits_revision_;
  bool enabled_{},geometry_ready_{},stop_requested_{};
  double linear_velocity_{},angular_velocity_{},linear_acceleration_{},angular_acceleration_{};
  double ik_position_tolerance_{},ik_orientation_tolerance_{};
  std::uint64_t generation_{},configuration_generation_{},sequence_{};
  mks_servo_can::Time settling_{},confirmed_{},last_position_{},brake_deadline_{};
  rclcpp::Service<robot_control_interfaces::srv::JogTcp>::SharedPtr service_;
  rclcpp::Publisher<Status>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr model_subscription_;
  rclcpp::Subscription<robot_control_interfaces::msg::RobotConfiguration>::SharedPtr configuration_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
