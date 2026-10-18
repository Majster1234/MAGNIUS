#pragma once
#include <array>
#include <thread>
#include "hardware_interface/system_interface.hpp"
#include "robot_can_diagnostics/diagnostics.hpp"

namespace robot_hardware {
class CanSystem : public hardware_interface::SystemInterface {
 public:
  ~CanSystem() override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
 private:
  void shutdown();
  mks_servo_can::BusConfig config_;
  std::shared_ptr<mks_servo_can::BusRuntime> runtime_;
  std::shared_ptr<robot_can_diagnostics::DiagnosticsNode> diagnostics_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread ros_thread_;
  mks_servo_can::ControlState state_;
  std::array<double, mks_servo_can::kAxes> positions_{}, commands_{};
  std::uint64_t sequence_{};
  bool active_{false};
};
}  // namespace robot_hardware
