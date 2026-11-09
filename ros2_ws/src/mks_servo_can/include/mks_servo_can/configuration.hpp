#pragma once
#include <array>
#include <cstdint>
#include <string>
namespace mks_servo_can {
// Jednostka prędkości MKS przy mikrokroku 128; mniejsze wartości są i tak zaokrąglane w górę do jednostki.
constexpr double kDefaultMinDitherRpm=.125, kMaxMinDitherRpm=10.;
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
  bool disable_enable_on_critical_fault{true};
  std::string ptp_lin_dispatch_mode{"streamed"};
  bool position_integral{false};
  // Minimalna prędkość F5 / dolna granica ditheringu [RPM wału] w PTP/LIN i JOG TCP.
  double min_dither_rpm{kDefaultMinDitherRpm};
  // Kod acc ramek F5 w PTP/LIN i JOG TCP (1–255); 0 = execution_acceleration_codes z parametrów wykonania.
  std::uint8_t f5_acceleration_code{0};
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
