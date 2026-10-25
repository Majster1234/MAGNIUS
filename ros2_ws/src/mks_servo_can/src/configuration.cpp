#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include <cmath>
#include <algorithm>

namespace mks_servo_can {
void CanBusManager::require_configuration() {
  configuration_ready_ = false; configuration_locked_ = true; configuration_stage_ = "boot";
  if (executor_.status.state == "ready") {
    executor_.status.state = "invalidated";
    executor_.status.message = "Konfiguracja wymaga uzgodnienia; przygotuj program ponownie";
  }
}
ConfigurationResult CanBusManager::configure(ConfigurationPhase phase, const std::string &operation,
    const std::string &expected, const RobotProfile &profile, Time now) {
  auto result = [&](bool accepted, const std::string &message) {
    return ConfigurationResult{accepted, message, profile_.revision, configuration_operation_, configuration_stage_, configuration_locked_};
  };
  if (phase == ConfigurationPhase::status) return result(true, "Stan konfiguracji");
  if (operation.empty() || operation.size() > 64) return result(false, "Nieprawidłowy identyfikator operacji");
  const bool bootstrap = phase == ConfigurationPhase::bootstrap;
  if (phase == ConfigurationPhase::prepare || bootstrap) {
    if (std::find(retired_configuration_operations_.begin(), retired_configuration_operations_.end(), operation) != retired_configuration_operations_.end())
      return result(false, "Operacja została wcześniej zakończona");
    if (configuration_operation_ == operation && configuration_stage_ != "boot")
      return result(configuration_stage_ == "prepared" || configuration_stage_ == "committed", "Operacja już obsłużona");
    if (bootstrap ? configuration_ready_ || configuration_stage_ != "boot" : configuration_locked_ || !configuration_ready_)
      return result(false, "Inna operacja lub nieuzgodniona konfiguracja");
    if (!bootstrap && expected != profile_.revision) return result(false, "Konflikt rewizji konfiguracji C++");
    if (active_ || manual_->busy() || fault_) return result(false, "Zapis wymaga nieaktywnego sterowania i braku awarii");
    if (profile.revision.empty() || profile.revision.size() > 64) return result(false, "Nieprawidłowa rewizja");
    auto candidate = profile;
    for (std::size_t i = 0; i < kAxes; ++i) {
      const auto &a = candidate.axes[i];
      if (profile_.axes[i].configured && !a.configured) return result(false, "Nie można usunąć profilu skonfigurowanej osi");
      if (a.configured && (!std::isfinite(a.gear_ratio) || a.gear_ratio <= 0 ||
          (a.direction != 1 && a.direction != -1) || !std::isfinite(a.offset) || !std::isfinite(a.lower) ||
          !std::isfinite(a.upper) || a.lower >= a.upper || !std::isfinite(a.velocity) || a.velocity <= 0 ||
          !std::isfinite(a.acceleration) || a.acceleration <= 0)) return result(false, "Nieprawidłowy profil osi");
      candidate.changed[i] = candidate.changed[i] || !(a == profile_.axes[i]) ||
          (a.configured && !manual_->calibration_valid(i + 1));
      const auto &device = devices_[i + 1];
      if (!bootstrap && candidate.changed[i] && device.detected && device.consecutive_timeouts < 3 &&
          !manual_->configuration_stopped(i + 1, now))
        return result(false, "J" + std::to_string(i + 1) + ": wymagany potwierdzony postój i ENABLE=0");
    }
    // Także rollback nie przywraca gotowości starego programu.
    if (executor_.status.state == "ready") {
      executor_.status.state = "invalidated";
      executor_.status.message = "Zapis konfiguracji unieważnił program; przelicz i przygotuj PTP ponownie";
    }
    previous_profile_ = profile_; previous_axes_ = config_.axes; candidate_profile_ = candidate;
    previous_calibration_invalid_ = calibration_invalid_; configuration_bootstrap_ = bootstrap;
    configuration_operation_ = operation; configuration_locked_ = true; configuration_stage_ = "prepared";
    scanning_ = false;
    if (!bootstrap) return result(true, "Konfiguracja przygotowana; sterowanie zablokowane");
    phase = ConfigurationPhase::commit;
  }
  if (phase == ConfigurationPhase::rollback && !configuration_locked_ && configuration_operation_ != operation) {
    // Tombstone dla PREPARE, którego ACK zaginął lub żądanie dotarło z opóźnieniem.
    // FINISH pozostawi identyfikator, więc spóźniony PREPARE/COMMIT nie wykona zmian.
    configuration_operation_ = operation; configuration_stage_ = "finished";
    retired_configuration_operations_[retired_configuration_cursor_++ % retired_configuration_operations_.size()] = operation;
    return result(true, "Anulowano operację, która nie zmieniła konfiguracji");
  }
  if (configuration_operation_ != operation) return result(false, "Operacja nie jest właścicielem blokady");
  if (phase == ConfigurationPhase::commit) {
    if (configuration_stage_ == "committed") return result(true, "Konfiguracja już zastosowana");
    if (configuration_stage_ != "prepared") return result(false, "Operacja zakończona lub wycofana; spóźniony COMMIT odrzucony");
    if (fault_ || active_ || manual_->busy() || !transport_->is_open())
      return result(false, "Stan magistrali lub sterowania zmienił się przed COMMIT");
    if (!bootstrap) for (std::size_t i = 0; i < kAxes; ++i) {
      const auto &device = devices_[i + 1];
      if (candidate_profile_.changed[i] && device.detected && device.consecutive_timeouts < 3 &&
          !manual_->configuration_stopped(i + 1, now)) return result(false, "Warunki postoju/ENABLE zmieniły się przed COMMIT");
    }
    profile_ = candidate_profile_;
    for (std::size_t i = 0; i < kAxes; ++i) if (profile_.changed[i]) calibration_invalid_[i] = bootstrap && profile_.calibration_invalid[i];
    for (std::size_t i = 0; i < kAxes; ++i) if (profile_.axes[i].configured) {
      const auto &p = profile_.axes[i]; auto &a = config_.axes[i];
      a.address = i + 1; a.gear_ratio = p.gear_ratio; a.direction = p.direction; a.offset = p.offset;
      a.lower = p.lower; a.upper = p.upper; a.max_velocity = p.velocity; a.max_acceleration = p.acceleration;
    }
    rebuild_axis_lookup();
    manual_->configuration_changed(profile_.changed);
    config_.commissioning_verified = config_.synchronization_verified = config_.stop_verified = false;
    configuration_stage_ = "committed";
    return result(true, "Parametry zastosowane; oczekiwanie na trwały zapis");
  }
  if (phase == ConfigurationPhase::rollback) {
    if (configuration_stage_ == "rolled_back") return result(true, "Wycofanie już potwierdzone");
    if (!configuration_locked_ || configuration_stage_ == "finished") return result(false, "Operacja już zakończona");
    profile_ = previous_profile_; config_.axes = previous_axes_; calibration_invalid_ = previous_calibration_invalid_;
    rebuild_axis_lookup();
    manual_->configuration_changed(candidate_profile_.changed);
    configuration_stage_ = "rolled_back";
    return result(true, "Poprzednie parametry przywrócone; blokada pozostaje aktywna");
  }
  if (phase == ConfigurationPhase::finish) {
    if (configuration_stage_ == "finished") return result(true, "Operacja już zakończona");
    if (configuration_stage_ != "committed" && configuration_stage_ != "rolled_back") return result(false, "Najpierw COMMIT lub ROLLBACK");
    configuration_ready_ = true; configuration_locked_ = false; configuration_stage_ = "finished";
    retired_configuration_operations_[retired_configuration_cursor_++ % retired_configuration_operations_.size()] = operation;
    if (configuration_bootstrap_) { configuration_bootstrap_ = false; if(scan_after_bootstrap_) start_scan(); }
    return result(true, "Konfiguracja uzgodniona");
  }
  return result(false, "Nieznana faza konfiguracji");
}
}  // namespace mks_servo_can
