#pragma once
#include <cmath>
#include <stdexcept>
namespace robot_can_diagnostics {
// Postój wału: enkoder magnetyczny i małe zawahania silnika dają szum pochodnej pozycji (filtr 100 ms)
// rzędu setnych RPM. 0,5 RPM wału ≈ 0,3° wału na 100 ms, czyli rząd tolerancji dojazdu.
inline constexpr double kStandstillShaftRpm=.5;
inline bool shaft_at_standstill(double rpm) {return std::isfinite(rpm) && std::abs(rpm)<=kStandstillShaftRpm;}
inline void validate_tcp_parameters(double jerk,double motor_tolerance_deg) {
  if(!std::isfinite(jerk) || jerk<=0 || jerk>10)throw std::invalid_argument("tcp_jog_max_jerk: wymagane (0, 10] m/s³");
  if(!std::isfinite(motor_tolerance_deg) || motor_tolerance_deg<.05 || motor_tolerance_deg>10)
    throw std::invalid_argument("motor_stop_tolerance_deg: wymagane [0.05, 10] stopnia wału silnika");
}
// Tolerancja dojazdu w stopniach wału silnika; uchyb przegubu przeliczany przez przełożenie osi.
inline bool tcp_stop_within(double joint_error_rad,double motor_tolerance_deg,double gear_ratio) {
  return std::isfinite(joint_error_rad) && std::isfinite(gear_ratio) && gear_ratio>0 &&
      std::abs(joint_error_rad)*gear_ratio<=motor_tolerance_deg*std::acos(-1.)/180+1e-12;
}
}
