#pragma once

#include "mks_servo_can/bus_manager.hpp"
#include <algorithm>

namespace mks_servo_can {
// Wieloobrotowe odczyty wału MKS: 16384 ticki/obrót, bez przełożeń przegubów.
class IdleEnableMonitor {
 public:
  static constexpr auto timeout = std::chrono::minutes(60);
  static constexpr auto observation_gap = std::chrono::milliseconds(250);

  void reset() { tracking_ = false; }

  bool observe(Time now, const std::array<std::int64_t, kAxes> &positions,
      bool eligible, unsigned enabled_mask) {
    if (!eligible || !enabled_mask) { reset(); return false; }
    if (!tracking_ || now < last_observed_ || now - last_observed_ > observation_gap ||
        enabled_mask != enabled_mask_) {
      start(now, positions, enabled_mask);
      return false;
    }
    last_observed_ = now;
    for (unsigned j = 0; j < kAxes; ++j) {
      minimum_[j] = std::min(minimum_[j], positions[j]);
      maximum_[j] = std::max(maximum_[j], positions[j]);
      // Ściśle ponad 2°. Zakres wykrywa także ruch tam i z powrotem.
      if (static_cast<double>(maximum_[j] - minimum_[j]) * 360.0 > 2.0 * 16384.0) {
        start(now, positions, enabled_mask);
        return false;
      }
    }
    return now - started_ >= timeout;
  }

 private:
  void start(Time now, const std::array<std::int64_t, kAxes> &positions, unsigned mask) {
    tracking_ = true; started_ = last_observed_ = now;
    minimum_ = maximum_ = positions; enabled_mask_ = mask;
  }
  bool tracking_{};
  unsigned enabled_mask_{};
  Time started_{}, last_observed_{};
  std::array<std::int64_t, kAxes> minimum_{}, maximum_{};
};
}
