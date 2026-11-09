#pragma once
#include <array>
#include <cmath>
#include <vector>
#include "mks_servo_can/manual_motor_controller.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace robot_can_diagnostics {
// Projekcja gotowego snapshotu; brak transportu i dodatkowych odczytów CAN.
inline std::vector<sensor_msgs::msg::JointState> preview_states(
    const mks_servo_can::ManualState &state, mks_servo_can::Time now,
    rclcpp::Time ros_now, std::array<mks_servo_can::Time, mks_servo_can::kAxes> &last) {
  std::vector<sensor_msgs::msg::JointState> messages;
  for (std::size_t i = 0; i < state.axes.size(); ++i) {
    const auto &a = state.axes[i];
    if (!a.available || !a.position.fresh(now, std::chrono::milliseconds(250)) ||
        !std::isfinite(a.position.value) || a.position.stamp <= last[i]) continue;
    const auto age = std::chrono::duration_cast<std::chrono::nanoseconds>(now - a.position.stamp).count();
    if (age > ros_now.nanoseconds()) continue;
    sensor_msgs::msg::JointState out;
    out.header.stamp = ros_now - rclcpp::Duration::from_nanoseconds(age);
    out.name = {"joint_" + std::to_string(i + 1)};
    if (a.configured && !a.joint_position.valid) continue;
    out.position = {a.configured ? a.joint_position.value : a.position.value * std::acos(-1.0) / 180.0};
    messages.push_back(std::move(out)); last[i] = a.position.stamp;
  }
  return messages;
}
}  // namespace robot_can_diagnostics
