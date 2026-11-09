#pragma once
#include <array>
#include <cstdint>
#include <vector>
namespace mks_servo_can {
struct AxisConfig;
struct MotorSample {
  std::int32_t ticks{};
  std::uint16_t rpm{};
  std::uint8_t acceleration{}, operation{};
  double position_error{}, rpm_error{};
  std::vector<std::uint8_t> payload;
};
// Pozycja absolutna F5; zerowa pochodna nigdy nie staje się przypadkowym STOP.
MotorSample compile_motor_sample(const AxisConfig &axis, double position, double velocity,
                                 double previous_position, double seconds, bool strict_speed = true);
}
