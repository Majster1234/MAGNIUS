#include "robot_can_diagnostics/diagnostics.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto parameters = std::make_shared<rclcpp::Node>("can_bus_parameters");
    mks_servo_can::BusConfig config;
    config.interface = parameters->declare_parameter("can_interface", "can0");
    config.bitrate = parameters->declare_parameter("bitrate", 500000);
    config.response_timeout = std::chrono::milliseconds(parameters->declare_parameter("response_timeout_ms", 20));
    config.cycle = std::chrono::milliseconds(parameters->declare_parameter("cycle_ms", 10));
    config.use_ack=parameters->declare_parameter("use_ack",false);
    config.rx_poll_timeout_ms=parameters->declare_parameter("rx_poll_timeout_ms",1);
    // SCHED_FIFO wątków IO/RX CAN (terminy SYNC); 0 = zwykły planista. Wymaga LimitRTPRIO w usłudze.
    config.io_realtime_priority=static_cast<int>(parameters->declare_parameter("can_io_realtime_priority",0));
    config.manual_max_rpm = parameters->declare_parameter("manual_max_rpm", 30);
    auto runtime = std::make_shared<mks_servo_can::BusRuntime>(config);
    auto node = std::make_shared<robot_can_diagnostics::DiagnosticsNode>(runtime, config);
    runtime->start(false);  // Skan uruchomi się raz, po uzgodnieniu konfiguracji.
    if(config.io_realtime_priority>0) {
      const auto realtime=runtime->realtime_status();
      if(realtime.empty()) RCLCPP_INFO(parameters->get_logger(),"Wątki IO/RX CAN: SCHED_FIFO %d",config.io_realtime_priority);
      else RCLCPP_WARN(parameters->get_logger(),"Wątki IO/RX CAN bez priorytetu RT: %s",realtime.c_str());
    }
    parameters.reset();
    // Dwa wątki: grupa sterowania (stan, JOG, STOP) nie czeka na programy i diagnostykę.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node); executor.spin(); runtime->shutdown();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_can_diagnostics"), "%s", e.what()); rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
