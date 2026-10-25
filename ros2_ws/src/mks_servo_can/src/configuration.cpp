#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

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
    auto resolved=configuration_stage_ == "prepared" ? candidate_profile_ : profile_;
    if(configuration_stage_!="prepared") resolved.calibration_invalid=calibration_invalid_;
    return ConfigurationResult{accepted, message, profile_.revision, configuration_operation_, configuration_stage_, configuration_locked_, resolved};
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
    std::array<bool,kAxes> anchored{};
    std::array<std::int64_t,kAxes> ticks{};
    std::array<std::uint64_t,kAxes> segments{};
    for (std::size_t i = 0; i < kAxes; ++i) {
      auto &a = candidate.axes[i];
      const auto &old = profile_.axes[i];
      if (profile_.axes[i].configured && !a.configured) return result(false, "Nie można usunąć profilu skonfigurowanej osi");
      if (a.configured && (!std::isfinite(a.gear_ratio) || a.gear_ratio <= 0 ||
          (a.direction != 1 && a.direction != -1) || !std::isfinite(a.offset) || !std::isfinite(a.lower) ||
          !std::isfinite(a.upper) || a.lower >= a.upper || !std::isfinite(a.velocity) || a.velocity <= 0 ||
          !std::isfinite(a.acceleration) || a.acceleration <= 0)) return result(false, "Nieprawidłowy profil osi");
      const bool ratio_changed=old.configured && a.configured && old.gear_ratio!=a.gear_ratio;
      const bool reference_changed=old.configured && a.configured &&
          (old.direction!=a.direction || std::abs(old.offset-a.offset)>1e-12);
      if(!bootstrap && ratio_changed && (reference_changed || profile.confirm_calibration[i]))
        return result(false,"J"+std::to_string(i+1)+": przełożenie i odniesienie wymagają osobnych zapisów");
      candidate.changed[i] = candidate.changed[i] || !(a == old) || profile.confirm_calibration[i];
      const auto &device=devices_[i+1];
      const bool live=device.detected && device.consecutive_timeouts<3 && device.position.fresh(now,std::chrono::milliseconds(60));
      if(!bootstrap && candidate.changed[i]) {
        // Znane ENABLE=1 nigdy nie jest traktowane jako oś odłączona.
        if((device.enable.fresh(now,std::chrono::milliseconds(300)) && device.enable.value) ||
            (live && !manual_->configuration_stopped(i+1,now)))
          return result(false,"J"+std::to_string(i+1)+": wymagany potwierdzony postój i ENABLE=0");
      }
      if(!bootstrap) {
        candidate.calibration_invalid[i]=calibration_invalid_[i];
        if(a.configured && candidate.changed[i]) {
          if(ratio_changed && manual_->calibration_valid(i+1) && live) {
            const auto current=position_to_joint(config_.axes[i],device.position.value);
            a.offset=current-a.direction*static_cast<double>(device.position.value)*(2*std::acos(-1.)/16384)/a.gear_ratio;
            if(current<a.lower || current>a.upper) return result(false,"Zachowany kąt poza nowymi limitami osi J"+std::to_string(i+1));
            candidate.calibration_invalid[i]=false;
            anchored[i]=true;
          } else if(ratio_changed || !live) candidate.calibration_invalid[i]=true;
          else if(!old.configured || reference_changed || profile.confirm_calibration[i]) candidate.calibration_invalid[i]=false;
          if(profile.confirm_calibration[i] && !live)
            return result(false,"Potwierdzenie odniesienia wymaga świeżego pomiaru i postoju");
          if(live) {anchored[i]=true;ticks[i]=device.position.value;segments[i]=shaft_segments_[i];}
        }
      }
      if(a.configured) {
        auto axis=config_.axes[i];axis.address=i+1;axis.gear_ratio=a.gear_ratio;axis.direction=a.direction;axis.offset=a.offset;
        axis.lower=a.lower;axis.upper=a.upper;axis.max_velocity=a.velocity;axis.max_acceleration=a.acceleration;
        try {
          joint_to_position(axis,axis.lower);joint_to_position(axis,axis.upper);
          const auto rpm=axis.max_velocity*axis.gear_ratio*60/(2*std::acos(-1.));
          if(rpm<1 || rpm>3000) throw std::invalid_argument("Prędkość osi poza zakresem MKS 1–3000 RPM");
          if(!bootstrap && live && !candidate.calibration_invalid[i]) {
            const auto q=position_to_joint(axis,device.position.value);
            if(q<a.lower || q>a.upper) throw std::invalid_argument("Bieżący kąt poza nowymi limitami");
          }
        } catch(const std::exception &e) {return result(false,"J"+std::to_string(i+1)+": "+e.what());}
      }
    }
    // Także rollback nie przywraca gotowości starego programu.
    if (executor_.status.state == "ready") {
      executor_.status.state = "invalidated";
      executor_.status.message = "Zapis konfiguracji unieważnił program; przelicz i przygotuj PTP ponownie";
    }
    previous_profile_ = profile_; previous_axes_ = config_.axes; candidate_profile_ = candidate;
    previous_calibration_invalid_ = calibration_invalid_; configuration_bootstrap_ = bootstrap;
    configuration_anchored_=anchored;configuration_ticks_=ticks;configuration_segments_=segments;
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
      if(configuration_anchored_[i] && (!device.detected || device.consecutive_timeouts>=3 ||
          !manual_->configuration_stopped(i+1,now) || shaft_segments_[i]!=configuration_segments_[i] ||
          std::abs(device.position.value-configuration_ticks_[i])>3))
        return result(false,"Odniesienie lub warunki postoju zmieniły się przed COMMIT");
      if(candidate_profile_.changed[i] && device.enable.fresh(now,std::chrono::milliseconds(300)) && device.enable.value)
        return result(false,"ENABLE zmieniło się przed COMMIT");
    }
    profile_ = candidate_profile_;
    calibration_invalid_=profile_.calibration_invalid;
    ++configuration_generation_;latest_.reset();streaming_states_={};previous_velocity_.fill(0);
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
    ++configuration_generation_;latest_.reset();streaming_states_={};previous_velocity_.fill(0);
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
