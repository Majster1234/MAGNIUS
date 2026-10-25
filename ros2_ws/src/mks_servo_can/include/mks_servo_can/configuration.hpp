#pragma once
#include <array>
#include <cstdint>
#include <string>
namespace mks_servo_can {
struct JointProfile {
  bool configured{};
  double gear_ratio{}, direction{}, offset{}, lower{}, upper{}, velocity{}, acceleration{};
  bool operator==(const JointProfile &o) const {
    return configured == o.configured && (!configured || (gear_ratio == o.gear_ratio && direction == o.direction &&
      offset == o.offset && lower == o.lower && upper == o.upper && velocity == o.velocity && acceleration == o.acceleration));
  }
};
struct RobotProfile {
  std::string revision{"initial"};
  std::array<JointProfile, 6> axes{};
  std::array<bool, 6> changed{};
  std::array<bool, 6> calibration_invalid{};
  std::array<bool, 6> confirm_calibration{};
};
struct JointConversion {
  double gear_ratio{1}, direction{1}, offset{};
  std::uint16_t address{};
  bool calibrated{};
  double position(std::int64_t ticks) const {
    return direction*static_cast<double>(ticks)*(2*3.14159265358979323846/16384)/gear_ratio+offset;
  }
};
enum class ConfigurationPhase { prepare, commit, rollback, finish, status, bootstrap };
struct ConfigurationResult {
  bool accepted{};
  std::string message, revision, operation_id, stage;
  bool locked{};
  RobotProfile resolved_configuration;
};
}  // namespace mks_servo_can
