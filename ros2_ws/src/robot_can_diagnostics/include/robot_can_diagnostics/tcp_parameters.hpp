#pragma once
#include <cmath>
#include <stdexcept>
namespace robot_can_diagnostics {
inline void validate_tcp_parameters(double jerk,double tolerance_deg) {
  if(!std::isfinite(jerk) || jerk<=0 || jerk>10)throw std::invalid_argument("tcp_jog_max_jerk: wymagane (0, 10] m/s³");
  if(!std::isfinite(tolerance_deg) || tolerance_deg<.02 || tolerance_deg>1)
    throw std::invalid_argument("joint_stop_tolerance_deg: wymagane [0.02, 1.0] stopnia przegubu");
}
inline bool tcp_stop_within(double error_rad,double tolerance_deg) {
  return std::isfinite(error_rad) && std::abs(error_rad)<=tolerance_deg*std::acos(-1.)/180+1e-12;
}
}
