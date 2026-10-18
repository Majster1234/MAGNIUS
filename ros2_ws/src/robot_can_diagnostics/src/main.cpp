#include "robot_can_diagnostics/diagnostics.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto parameters = std::make_shared<rclcpp::Node>("can_bus_parameters");
    mks_servo_can::BusConfig config;
    config.interface = parameters->declare_parameter("can_interface", "can0");
    config.bitrate = parameters->declare_parameter("bitrate", 500000);
    config.response_timeout = std::chrono::milliseconds(parameters->declare_parameter("response_timeout_ms", 20));
    config.cycle = std::chrono::milliseconds(parameters->declare_parameter("cycle_ms", 20));
    config.manual_max_rpm = parameters->declare_parameter("manual_max_rpm", 30);
    auto runtime = std::make_shared<mks_servo_can::BusRuntime>(config);
    auto node = std::make_shared<robot_can_diagnostics::DiagnosticsNode>(runtime, config);
    runtime->start(false);  // Skan uruchomi się raz, po uzgodnieniu konfiguracji.
    parameters.reset(); rclcpp::spin(node); runtime->shutdown();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_can_diagnostics"), "%s", e.what()); rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
