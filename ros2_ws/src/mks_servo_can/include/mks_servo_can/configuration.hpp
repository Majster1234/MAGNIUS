#pragma once
#include <array>
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
};
enum class ConfigurationPhase { prepare, commit, rollback, finish, status, bootstrap };
struct ConfigurationResult {
  bool accepted{};
  std::string message, revision, operation_id, stage;
  bool locked{};
};
}  // namespace mks_servo_can
