#include "mks_servo_can/manual_motor_controller.hpp"
#include "mks_servo_can/trace.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

using namespace std::chrono_literals;
namespace mks_servo_can {
namespace {
double degrees(std::int64_t ticks) { return static_cast<double>(ticks) * 360.0 / 16384.0; }
constexpr double kJogRestToleranceDegrees = 1.0;
}
bool ManualMotorController::stopped(unsigned axis, Time now, std::chrono::milliseconds window) const {
  const auto &d = bus_.devices_[axis];
  return d.speed.fresh(now, 100ms) &&
      d.motor_state.fresh(now, 150ms) && d.motor_state.value == 1 && position_stable(axis, now, window);
}
bool ManualMotorController::position_stable(unsigned axis, Time now, std::chrono::milliseconds window) const {
  const auto &p = bus_.devices_[axis].position; const auto &anchor = stationary_anchor_[axis - 1];
  return p.fresh(now, 60ms) && anchor.valid && p.stamp >= anchor.stamp + window &&
      std::abs(p.value - anchor.value) <= 3;
}
bool ManualMotorController::jog_stopped(unsigned axis, Time now) const {
  const auto &d = bus_.devices_[axis]; const auto &anchor = jog_rest_anchor_[axis - 1];
  return d.speed.fresh(now, 100ms) && d.motor_state.fresh(now, 150ms) && d.motor_state.value == 1 &&
      d.position.fresh(now, 60ms) && anchor.valid && d.position.stamp >= anchor.stamp + 60ms &&
      std::abs(degrees(d.position.value - anchor.value)) <= kJogRestToleranceDegrees;
}
void ManualMotorController::observe_positions(Time now) {
  for (unsigned axis = 1; axis <= kAxes; ++axis) {
    const auto &d = bus_.devices_[axis]; const auto &p = d.position;
    auto &anchor = stationary_anchor_[axis - 1]; auto &jog_anchor = jog_rest_anchor_[axis - 1];
    auto &previous = velocity_previous_[axis - 1]; auto &velocity = velocity_[axis - 1];
    const auto feedback_age=bus_.active_ && bus_.source_==Source::program ? kProgramFeedbackTimeout : 60ms;
    if (!p.fresh(now, feedback_age)) { anchor.valid = jog_anchor.valid = previous.valid = velocity.valid = false; continue; }
    if (!anchor.valid || std::abs(p.value - anchor.value) > 3) anchor = p;
    // Osobne pasmo dla JOG: drobne drgania nie kasują potwierdzonego postoju.
    // Odniesienie nie podąża za każdą próbką, więc sumujący się ruch przekroczy pasmo.
    if (!d.motor_state.fresh(now, 150ms) || d.motor_state.value != 1 ||
        (axis == axis_ && (phase_ == Phase::starting || phase_ == Phase::moving))) jog_anchor.valid = false;
    else if (!jog_anchor.valid || std::abs(degrees(p.value - jog_anchor.value)) > kJogRestToleranceDegrees)
      jog_anchor = p;
    if (previous.valid && p.stamp > previous.stamp) {
      const auto dt = std::chrono::duration<double>(p.stamp - previous.stamp).count();
      const auto rpm = static_cast<double>(p.value - previous.value) * 60.0 / 16384.0 / dt;
      const auto filtered = velocity.valid ? velocity.value + dt / (0.1 + dt) * (rpm - velocity.value) : rpm;
      velocity.set(filtered, p.stamp);
    }
    if (!previous.valid || p.stamp > previous.stamp) previous = p;
  }
}
bool ManualMotorController::ready(unsigned axis, Time now) const {
  if (!axis || axis > kAxes) return false;
  if (!bus_.configuration_ready_ || bus_.configuration_locked_ ||
      (bus_.profile_.axes[axis - 1].configured && (!calibrated_[axis - 1] || !profile_verified_[axis - 1]))) return false;
  const auto &d = bus_.devices_[axis];
  return prepared_[axis - 1] && d.detected && d.identity.model == Model::kServo42D &&
      d.position.fresh(now, 60ms) && d.enable.fresh(now, 300ms) && d.enable.value &&
      d.stall.fresh(now, 300ms) && !d.stall.value && d.mode.valid && d.mode.value == 5;
}
bool ManualMotorController::begin(unsigned axis, Operation op, Time now) {
  if (!axis || axis > kAxes || !bus_.configuration_ready_ || bus_.configuration_locked_ ||
      (busy() && !(group_step_ && phase_==Phase::idle)) || bus_.active_ || bus_.fault_ || !bus_.transport_->is_open()) return false;
  const auto &d = bus_.devices_[axis];
  if (!d.detected || d.identity.model != Model::kServo42D || !d.position.fresh(now, 60ms)) return false;
  bus_.scanning_ = false;
  axis_ = axis; operation_ = op; phase_ = Phase::sequence;
  step_ = step_count_ = 0; waiting_ = false; disable_after_ = false; retry_at_ = {};
  operation_deadline_ = now + 2s;
  message_ = "Wykonywanie operacji osi J" + std::to_string(axis);
  return true;
}
void ManualMotorController::add(std::vector<std::uint8_t> data, std::size_t length, int expected) {
  if (step_count_ == steps_.size()) throw std::logic_error("Przekroczony limit sekwencji napędu");
  steps_[step_count_++] = {std::move(data), length, expected};
}
bool ManualMotorController::set_enable(unsigned axis, bool enabled, Time now) {
  if(group_active_ && !group_step_) return false;
  if (!enabled && phase_!=Phase::idle) {
    if (axis != axis_ || (phase_ != Phase::starting && phase_ != Phase::moving && phase_ != Phase::stopping)) return false;
    disable_after_ = true; stop(now, "Hamowanie przed wyłączeniem ENABLE"); return true;
  }
  if (!begin(axis, enabled ? Operation::enable : Operation::disable, now)) return false;
  prepared_[axis - 1] = false;
  if (enabled) {
    // Ręczny F6 wymaga 4A=0; stare potwierdzenie synchronicznego ruchu sześciu osi traci ważność.
    bus_.config_.synchronization_verified = false;
    // Odczyt 32 działa także w trybie CR. Nigdy nie przygotowujemy poruszającego się wału.
    add(request(axis, 0x32), 4, 0);
    add(request(axis, 0x8c, {0, 0}));add(parameter_request(axis,0x8c),4,0);
    add(request(axis, 0x4a, {0})); add(parameter_request(axis, 0x4a), 3, 0);
    add(request(axis, 0x82, {5})); add(parameter_request(axis, 0x82), 3, 5);
    add(enable_request(axis, false)); add(request(axis, 0x3a), 3, 0);
    // Przy ENABLE=0 firmware odrzuca także F6 z prędkością 0. F7 czyści
    // poprzedni ruch bez załączania momentu (potwierdzone na 1.0.9).
    add(stop_request(axis));
    add(request(axis, 0x32), 4, 0); add(request(axis, 0xf1), 3, 1);
    add(parameter_request(axis, 0x84), 3, -1);
    add(request(axis, 0x98, {0, 0, 1, 44})); add(parameter_request(axis, 0x98), 6, 300);
    add(enable_request(axis, true)); add(request(axis, 0x3a), 3, 1);
  } else {
    // Przy wyłączaniu również najpierw potwierdzamy postój.
    add(request(axis, 0x32), 4, 0); add(request(axis, 0xf1), 3, 1);
    add(enable_request(axis, false)); add(request(axis, 0x3a), 3, 0);
  }
  return true;
}
bool ManualMotorController::set_enable_all(bool enabled, Time now) {
  if(busy() || bus_.active_ || bus_.fault_ || bus_.scanning_ || !bus_.configuration_ready_ || bus_.configuration_locked_ || !bus_.transport_->is_open()) return false;
  std::array<bool,kAxes> targets{};
  for(unsigned j=0;j<kAxes;++j) {
    const auto &device=bus_.devices_[j+1];
    if(!device.detected) continue;
    if(!device.position.fresh(now,60ms) || !position_stable(j+1,now,100ms)) return false;
    targets[j]=true;
  }
  if(std::none_of(targets.begin(),targets.end(),[](bool value){return value;})) return false;
  group_active_=true;group_enabled_=enabled;group_committing_=false;group_targets_=targets;group_done_.fill(false);group_axis_=0;
  message_=enabled ? "Załączanie wszystkich dostępnych napędów" : "Wyłączanie wszystkich dostępnych napędów";
  return true;
}
void ManualMotorController::group_tick(Time now) {
  if(!group_active_ || phase_!=Phase::idle) return;
  if(group_committing_) {
    if(group_axis_) {
      if(now>=operation_deadline_) {fail("Nie potwierdzono ENABLE J"+std::to_string(group_axis_)+"; sekwencja przerwana");return;}
      const auto &enable=bus_.devices_[group_axis_].enable;
      if(!enable.fresh(now,300ms) || enable.stamp<group_sent_ || enable.value!=group_enabled_) return;
      group_done_[group_axis_-1]=true;group_axis_=0;
    }
    for(unsigned j=0;j<kAxes;++j) if(group_targets_[j] && !group_done_[j]) {
      if(now<group_next_) return;
      if(!bus_.devices_[j+1].position.fresh(now,60ms) || !position_stable(j+1,now,100ms)) {
        fail("Zmiana lub utrata pozycji przed ENABLE J"+std::to_string(j+1)+"; sekwencja przerwana");return;
      }
      const double cost=2*can_frame_bits(3);
      const double budget=bus_.config_.bitrate*std::chrono::duration<double>(bus_.config_.cycle).count()*std::min(kPlannedUtilization,bus_.config_.utilization_limit);
      if(bus_.spent_bits_+cost>budget) return;
      bus_.spent_bits_+=cost;group_axis_=j+1;group_sent_=now;group_next_=now+200ms;operation_deadline_=now+2s;
      bus_.transport_->send(group_axis_,enable_request(group_axis_,group_enabled_));++bus_.stats_.tx;
      bus_.next_enable_[group_axis_]={};
      message_="Wysłano ENABLE J"+std::to_string(group_axis_)+"; potwierdzenie i odstęp 200 ms przed kolejną osią";
      return;
    }
    group_active_=false;
    message_=group_enabled_ ? "ENABLE potwierdzony dla wszystkich dostępnych napędów" : "Wyłączenie potwierdzone dla wszystkich dostępnych napędów";
    return;
  }
  for(unsigned j=0;j<kAxes;++j) if(group_targets_[j] && !group_done_[j]) {
    const auto &device=bus_.devices_[j+1];
    if(device.enable.fresh(now,300ms) && device.enable.value==group_enabled_ && (!group_enabled_ || prepared_[j])) {group_done_[j]=true;continue;}
    group_step_=true;const bool accepted=set_enable(j+1,group_enabled_,now);group_step_=false;
    if(!accepted) fail("Przerwano zbiorcze ENABLE na J"+std::to_string(j+1)+"; sprawdź stan każdej osi");
    else {
      // Ostatnie F3/3A odkładamy do sekwencji z odstępem co najmniej 200 ms.
      step_count_-=2;
      message_="Przygotowanie zbiorczego ENABLE: J"+std::to_string(j+1);
    }
    return;
  }
  group_committing_=true;group_done_.fill(false);group_next_=now;group_axis_=0;
  message_="Napędy przygotowane; sekwencja ENABLE co najmniej co 200 ms";
}
bool ManualMotorController::group_reply(unsigned axis,const std::vector<std::uint8_t> &data,Time stamp) {
  if(!group_active_ || !group_committing_ || axis<1 || axis>kAxes || axis!=group_axis_ || stamp<group_sent_ ||
      !validate_response(axis,data,0xf3,3).valid) return false;
  if(data[1]!=1) fail("Napęd J"+std::to_string(axis)+" odrzucił zbiorcze ENABLE");
  return true;
}
bool ManualMotorController::zero(unsigned axis, Time now) {
  if (!axis || axis > kAxes || !stopped(axis, now, 100ms) || !begin(axis, Operation::zero, now)) return false;
  add(request(axis, 0x32), 4, 0); add(request(axis, 0xf1), 3, 1);
  add(zero_request(axis)); add(request(axis, 0x31), 8, 0);
  return true;
}
double ManualMotorController::brake_distance(unsigned axis, double rpm, Time now) const {
  const auto scale = speed_scale(microstep_[axis - 1]);
  const auto acceleration = scale / (255.0 * 0.00005);  // RPM/s, acc=1
  const auto age = std::max(0.0, std::chrono::duration<double>(now - bus_.devices_[axis].position.stamp).count());
  return 6.0 * rpm * rpm / (2.0 * acceleration) + 6.0 * rpm * (age + 0.06) + 1.0;
}
void ManualMotorController::configuration_changed(const std::array<bool, kAxes> &changed) {
  for (std::size_t i = 0; i < kAxes; ++i) if (changed[i]) {
    prepared_[i] = profile_verified_[i] = false;
    calibrated_[i] = bus_.profile_.axes[i].configured && !bus_.calibration_invalid_[i];
    configuration_messages_[i] = calibrated_[i] ? "Wymagane przygotowanie i weryfikacja napędu" :
        bus_.profile_.axes[i].configured ? "Odniesienie niepotwierdzone — potwierdź kalibrację osi" : "JOG wału — brak pełnej konfiguracji przegubu";
  }
}
bool ManualMotorController::configuration_stopped(unsigned axis, Time now) const {
  const auto &device = bus_.devices_[axis];
  return device.enable.fresh(now, 300ms) && !device.enable.value && position_stable(axis, now, 100ms);
}
bool ManualMotorController::limit_reached(unsigned axis, int direction, double rpm, Time now) const {
  const auto motor = degrees(bus_.devices_[axis].position.value);
  const auto distance = brake_distance(axis, rpm, now);
  if (direction * motor + distance >= 3600) return true;
  const auto &profile = bus_.profile_.axes[axis - 1];
  if (!profile.configured) return false;
  const auto q = (profile.direction * motor / profile.gear_ratio) * std::acos(-1.0) / 180.0 + profile.offset;
  const auto brake = distance * std::acos(-1.0) / 180.0 / profile.gear_ratio;
  return direction * profile.direction > 0 ? q + brake >= profile.upper : q - brake <= profile.lower;
}
bool ManualMotorController::jog(JogAction action, unsigned axis, int direction, unsigned rpm,
    const std::string &session, std::uint64_t sequence, Time issued, Time now) {
  return jog_impl(action, axis, direction, rpm, session, sequence, issued, now, false);
}
bool ManualMotorController::jog_joint(JogAction action, unsigned axis, int direction, double velocity,
    const std::string &revision, const std::string &session, std::uint64_t sequence, Time issued, Time now) {
  if (action != JogAction::start) {
    if (action != JogAction::stop && revision != bus_.profile_.revision) return false;
    return jog_impl(action, axis, direction, 0, session, sequence, issued, now, true);
  }
  if (!axis || axis > kAxes || revision != bus_.profile_.revision || !std::isfinite(velocity) || velocity <= 0) return false;
  const auto &profile = bus_.profile_.axes[axis - 1];
  if (!profile.configured || velocity > profile.velocity || !microstep_[axis - 1]) return false;
  const auto scale = speed_scale(microstep_[axis - 1]);
  const auto requested_rpm = velocity * profile.gear_ratio * 60 / (2 * std::acos(-1.0));
  const auto raw = std::floor(std::min<double>(requested_rpm, bus_.config_.manual_max_rpm) / scale + 1e-10);
  if (raw < 1 || raw > 3000) return false;
  const double actual_rpm = raw * scale;
  if (actual_rpm > requested_rpm + 1e-9) return false;
  return jog_impl(action, axis, direction * static_cast<int>(profile.direction), actual_rpm, session, sequence, issued, now, true);
}
bool ManualMotorController::jog_impl(JogAction action, unsigned axis, int direction, double rpm,
    const std::string &session, std::uint64_t sequence, Time issued, Time now, bool joint) {
  if (session.empty() || session.size() > 64 || !sequence || issued > now || now - issued >= 100ms) return false;
  if (action == JogAction::stop) {
    // STOP może dotrzeć przed opóźnionym START. Bariera czasu uniemożliwia jego wykonanie później.
    last_stop_ = std::max(last_stop_, issued);
    if (session != owner_ || axis != axis_ || sequence <= sequence_) return false;
    sequence_ = sequence; stop(now, "Hamowanie po puszczeniu przycisku"); return true;
  }
  if (action == JogAction::keepalive) {
    if (session != owner_ || axis != axis_ || sequence <= sequence_ ||
        (phase_ != Phase::starting && phase_ != Phase::moving)) return false;
    sequence_ = sequence; lease_ = issued; return true;
  }
  if (issued <= last_stop_ || busy() || bus_.active_ || bus_.fault_ || !ready(axis, now) || !jog_stopped(axis, now) ||
      bus_.profile_.axes[axis - 1].configured != joint ||
      (direction != -1 && direction != 1) || !std::isfinite(rpm) || rpm <= 0 || rpm > bus_.config_.manual_max_rpm) return false;
  const auto raw = std::lround(rpm / speed_scale(microstep_[axis - 1]));
  if (raw < 1 || raw > 3000 || std::abs(raw * speed_scale(microstep_[axis - 1]) - rpm) > .001) {
    message_ = "Wybrana prędkość nie jest reprezentowalna dla mikrokroku napędu";
    return false;
  }
  if (limit_reached(axis, direction, rpm, now)) return false;
  if (!begin(axis, Operation::jog, now)) return false;
  owner_ = session; sequence_ = sequence; lease_ = issued; rpm_ = rpm; direction_ = direction;
  phase_ = Phase::starting;
  jog_rest_anchor_[axis - 1].valid = false;
  add(speed_request(axis, direction > 0, static_cast<std::uint16_t>(raw)), 3, -2);
  message_ = "JOG osi J" + std::to_string(axis); return true;
}
void ManualMotorController::stop(Time now, const std::string &reason) {
  if (phase_ == Phase::stopping || phase_ == Phase::fault || phase_ == Phase::idle) return;
  trace_note("manual_stop", reason, axis_);
  phase_ = Phase::stopping; message_ = reason; last_stop_ = now;
  waiting_ = stop_sent_ = false;
  const auto &speed = bus_.devices_[axis_].speed;
  const auto rpm = std::max<double>(rpm_, speed.valid ? std::abs(speed.value) : 0);
  const auto scale = speed_scale(microstep_[axis_ - 1]);
  // F1 kończy rampę wcześniej niż ustala się rzeczywista pozycja wału.
  stop_deadline_ = now + std::chrono::milliseconds(static_cast<int>(rpm / scale * 12.75 + 2000));
  for (auto &p : bus_.pending_) if (p.used && p.address == axis_) p.used = false;
  bus_.quarantine_[axis_] = {};
}
void ManualMotorController::fail(const std::string &message) {
  trace_note("manual_fault", message, axis_);
  // Jeden ostatni STOP, potem cisza całej magistrali: diagnostyka nie maskuje 98H.
  if (phase_ != Phase::fault && axis_ && bus_.transport_->is_open()) {
    try { bus_.transport_->send(axis_, speed_request(axis_, false, 0)); ++bus_.stats_.tx; } catch (...) {}
  }
  group_active_=false;phase_ = Phase::fault; waiting_ = false; prepared_.fill(false); message_ = message;
  bus_.fault(message);
}
void ManualMotorController::transport_fault() {
  group_active_=false;
  if (busy() && phase_ != Phase::fault) {
    phase_ = Phase::fault; prepared_.fill(false); message_ = "Utrata komunikacji — stan ruchu nieznany";
  }
}
void ManualMotorController::timeout() { fail("Brak potwierdzenia operacji ręcznej — wymagany reset CAN"); }
void ManualMotorController::complete(Time) {
  if(group_active_) group_done_[axis_-1]=true;
  if (operation_ == Operation::enable) {
    prepared_[axis_ - 1] = true;
    bus_.next_speed_[axis_] = bus_.next_motor_[axis_] = bus_.next_enable_[axis_] = bus_.next_stall_[axis_] = {};
  }
  if (operation_ == Operation::zero) {
    bus_.config_.commissioning_verified = false;
    if (bus_.profile_.axes[axis_ - 1].configured) {
      calibrated_[axis_ - 1] = prepared_[axis_ - 1] = false;
      bus_.calibration_invalid_[axis_ - 1] = true;
      configuration_messages_[axis_ - 1] = "Zerowano wał; zatwierdź ponownie offset w konfiguracji";
    }
    stationary_anchor_[axis_ - 1].valid = jog_rest_anchor_[axis_ - 1].valid =
        velocity_previous_[axis_ - 1].valid = velocity_[axis_ - 1].valid = false;
  }
  message_ = group_active_ ? "Przygotowano napęd do zbiorczego ENABLE" : operation_ == Operation::zero ? "Pozycja zero potwierdzona" : "Operacja ENABLE potwierdzona";
  phase_ = Phase::idle; waiting_ = false;
}
void ManualMotorController::reply(const std::vector<std::uint8_t> &data, Time stamp) {
  if (!waiting_) return;
  waiting_ = false;
  if (phase_ == Phase::stopping) {
    if (data[1] != 1 && data[1] != 2) { fail("Napęd odrzucił zatrzymanie JOG: " + message_); return; }
    stop_sent_ = true; stop_at_ = stamp; return;
  }
  if (step_ >= step_count_) return;
  const auto &s = steps_[step_];
  std::int64_t value = 0;
  for (std::size_t i = 1; i + 1 < data.size(); ++i) value = (value << 8) | data[i];
  auto &d = bus_.devices_[axis_];
  if (data[0] == 0x32) { const auto v = parse_speed(axis_, data); if (!v) { fail("Błędny odczyt prędkości"); return; } d.speed.set(*v, stamp); value = *v; }
  if (data[0] == 0x31) { const auto v = parse_position(axis_, data); if (!v) { fail("Błędny odczyt pozycji"); return; } d.position.set(*v, stamp); value = *v; }
  if (data[0] == 0x32 && s.expected == 0) {
    if (!position_stable(axis_, stamp)) { retry_at_ = stamp + 20ms; return; }
    // 32H potrafi zachować ostatnią prędkość po postoju (potwierdzone na 1.0.9).
    // Warunek fizyczny pochodzi z kolejnych pozycji; w SR dodatkowo sprawdzamy F1.
    value = 0;
  }
  if (data[0] == 0x31 && s.expected == 0 && std::abs(value) <= 3) value = 0;
  if (s.expected >= 0 && value != s.expected) { fail("Napęd nie potwierdził warunku operacji: " + std::to_string(data[0])); return; }
  if (s.expected == -2 && value != 1 && value != 2) { fail("Napęd odrzucił komendę prędkości"); return; }
  // ACK zmiany zera rozpoczyna segment również wtedy, gdy późniejszy pomiar
  // nie potwierdzi tolerancji operacji. Trace zachowuje rzeczywiste ticki.
  if(data[0]==0x92 && operation_==Operation::zero) bus_.shaft_reference_changed(axis_);
  if (s.data.front() == 0) {
    if (data[0] == 0x84) {
      // Zero w odpowiedzi jest niejednoznaczne (256 w polu uint8). Nie zgadujemy.
      try { speed_scale(static_cast<std::uint16_t>(value)); } catch (const std::exception &e) { fail(e.what()); return; }
      microstep_[axis_ - 1] = static_cast<std::uint16_t>(value);d.microstep.set(static_cast<std::uint16_t>(value),stamp);
      const auto &profile = bus_.profile_.axes[axis_ - 1];
      if (profile.configured) {
        const auto acceleration = speed_scale(microstep_[axis_ - 1]) / .01275 * 2 * std::acos(-1.0) / 60 / profile.gear_ratio;
        if (!calibrated_[axis_ - 1] || acceleration > profile.acceleration + 1e-9) {
          if (!calibrated_[axis_ - 1]) {
            configuration_messages_[axis_ - 1] = "Wymagane ponowne zatwierdzenie offsetu";
          } else {
            std::ostringstream description;
            description << std::fixed << std::setprecision(1)
                << "JOG zablokowany: minimalne przyspieszenie MKS przy acc=1 wynosi "
                << acceleration * 180 / std::acos(-1.0) << " °/s², limit konfiguracji: "
                << profile.acceleration * 180 / std::acos(-1.0) << " °/s². Przełożenie: "
                << profile.gear_ratio << ". Większe acc oznacza szybszą rampę.";
            configuration_messages_[axis_ - 1] = description.str();
          }
          message_ = configuration_messages_[axis_ - 1]; phase_ = Phase::idle; prepared_[axis_ - 1] = false;
          return;  // Sekwencja kończy się przy ENABLE=0, bez awarii pozostałych osi.
        }
        profile_verified_[axis_ - 1] = true;
        configuration_messages_[axis_ - 1] = "JOG przegubu — profil zweryfikowany";
      }
    }
    if (data[0] == 0x82) d.mode.set(static_cast<std::uint8_t>(value), stamp);
    if (data[0] == 0x8c) (d.responses_known=true,d.responses_disabled=data.size()==4 && data[1]==0 && data[2]==0);
    if (data[0] == 0x4a) (d.synchronization_known=true,d.synchronization_disabled=value==0);
  }
  if (data[0] == 0x3a) d.enable.set(value != 0, stamp);
  if (data[0] == 0xf1) d.motor_state.set(static_cast<std::uint8_t>(value), stamp);
  ++step_;
  if (step_ == step_count_) {
    if (operation_ == Operation::jog) phase_ = Phase::moving;
    else complete(stamp);
  }
}
void ManualMotorController::tick(Time now) {
  observe_positions(now);
  group_tick(now);
  if (!busy() || phase_ == Phase::fault) return;
  if (now >= next_trace_) {
    const auto &d = bus_.devices_[axis_];
    const auto age = [now](const auto &s) { return s.stamp == Time{} ? -1 : trace_ns(now) - trace_ns(s.stamp); };
    TraceEvent e; e.event = "manual_feedback"; e.axis = axis_; e.flags = static_cast<unsigned>(phase_);
    e.position_age_ns = age(d.position); e.speed_age_ns = age(d.speed);
    e.enable_age_ns = age(d.enable); e.stall_age_ns = age(d.stall);
    e.duration_ns = age(d.motor_state); e.request_ns = trace_ns(lease_); trace(e);
    next_trace_ = now + 20ms;
  }
  if (phase_ == Phase::starting || phase_ == Phase::moving) {
    if (now < lease_ || now - lease_ >= 200ms) stop(now, "Brak podtrzymania JOG — hamowanie");
    else if (!ready(axis_, now) || !bus_.devices_[axis_].speed.fresh(now, 100ms) ||
        !bus_.devices_[axis_].motor_state.fresh(now, 150ms)) {
      const auto &d = bus_.devices_[axis_];
      const auto ms = [now](const auto &s) { return s.valid ? std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(now - s.stamp).count()) : "brak"; };
      stop(now, "Nieaktualne dane/ENABLE/stall: pozycja=" + ms(d.position) + " ms, prędkość=" + ms(d.speed) +
        " ms, stan=" + ms(d.motor_state) + " ms, ENABLE=" + ms(d.enable) + " ms/" + std::to_string(d.enable.value) +
        ", stall=" + ms(d.stall) + " ms/" + std::to_string(d.stall.value));
    }
    else {
      const auto &v = velocity_[axis_ - 1];
      const auto actual = v.fresh(now, 100ms) ? std::abs(v.value) : static_cast<double>(rpm_);
      if (limit_reached(axis_, direction_, std::max<double>(rpm_, actual), now))
        stop(now, "Hamowanie przed limitem osi");
    }
  }
  if (phase_ == Phase::stopping) {
    if (now >= stop_deadline_) { fail("Nie potwierdzono zatrzymania — stan ruchu nieznany"); return; }
    if (!stop_sent_ && !waiting_) {
      if (bus_.send_manual(axis_, speed_request(axis_, false, 0), 0, now, true)) {
        stop_sent_=true;stop_at_=now;waiting_=false;
      }
    }
    const auto &d = bus_.devices_[axis_];
    // Stabilność z kolejnych pozycji 50 Hz, bez dodatkowych dwóch cykli 32H.
    // Wymagamy pomiarów po nadaniu STOP, nigdy samego upływu czasu.
    if (stop_sent_ && jog_stopped(axis_, now) && d.position.stamp >= stop_at_ + 60ms &&
        d.speed.stamp > stop_at_ && d.motor_state.stamp > stop_at_) {
      phase_ = Phase::idle; last_stop_ = now; owner_.clear(); message_ = "Zatrzymanie potwierdzone";
      trace_note("manual_stop_confirmed", "", axis_);
      if (disable_after_) { disable_after_ = false; set_enable(axis_, false, now); }
    }
    return;
  }
  if (phase_ == Phase::moving) return;
  if (now >= operation_deadline_) { fail("Przekroczony czas operacji ręcznej"); return; }
  if (!waiting_ && step_ < step_count_ && now >= retry_at_) {
    const auto &s = steps_[step_];
    // Zapis konfiguracji/zera trwa dłużej od odczytu (92H: ~27 ms na sprzęcie).
    const auto opcode = s.data.front();
    const auto timeout = opcode == 0x92 || opcode == 0x82 || opcode == 0x98 || opcode == 0x4a || opcode == 0x8c ? 100ms : 0ms;
    if (bus_.send_manual(axis_, s.data, s.length, now, false, timeout)) {
      if(opcode==0xf5 || opcode==0xf6 || opcode==0xf7 || opcode==0xfe) {
        waiting_=false;++step_;
        if(step_==step_count_) {
          if(operation_==Operation::jog) phase_=Phase::moving;
          else complete(now);
        }
      } else waiting_=true;
    }
  }
}
ManualState ManualMotorController::snapshot(Time now) const {
  ManualState result; result.busy = busy() || bus_.active_ || bus_.configuration_locked_ || !bus_.configuration_ready_; result.fault = bus_.fault_;
  result.configuration_revision = bus_.profile_.revision;
  result.axis = group_active_ ? 0 : busy() ? axis_ : 0; result.max_rpm = bus_.config_.manual_max_rpm; result.message = message_;
  result.operation = phase_ == Phase::fault ? "fault" : phase_ == Phase::stopping ? "stopping" :
      phase_ == Phase::starting || phase_ == Phase::moving ? "jog" : phase_ == Phase::sequence ? "operation" : "idle";
  if(group_active_) result.operation=group_enabled_ ? "enable_all" : "disable_all";
  for (unsigned axis = 1; axis <= kAxes; ++axis) {
    auto &a = result.axes[axis - 1]; const auto &d = bus_.devices_[axis];
    a.axis = axis; a.available = !bus_.fault_ && bus_.transport_->is_open() && d.detected &&
        d.identity.model == Model::kServo42D && d.position.fresh(now, 250ms);
    a.prepared = prepared_[axis - 1];
    a.enable = d.enable; a.enable.valid = d.enable.fresh(now, 500ms) && !bus_.fault_;
    a.position = {degrees(d.position.value), d.position.stamp, d.position.fresh(now, 250ms) && !bus_.fault_};
    const auto &profile = bus_.profile_.axes[axis - 1];
    a.configured = profile.configured; a.calibration_valid = calibrated_[axis - 1];
    a.configuration_message = configuration_messages_[axis - 1];
    if (profile.configured) {
      a.joint_lower = profile.lower; a.joint_upper = profile.upper;
      a.joint_position = {CanBusManager::position_to_joint(bus_.config_.axes[axis-1],d.position.value),
          a.position.stamp, a.position.valid && a.calibration_valid};
      const auto scale = microstep_[axis - 1] ? speed_scale(microstep_[axis - 1]) : 0;
      a.minimum_velocity = scale * 2 * std::acos(-1.0) / 60 / profile.gear_ratio;
      a.maximum_velocity = std::min(profile.velocity, std::floor(bus_.config_.manual_max_rpm / (scale ? scale : 1)) * (scale ? scale : 1) * 2 * std::acos(-1.0) / 60 / profile.gear_ratio);
    }
    // Prędkość w układzie 31: pochodna pozycji, filtr 100 ms, stała pamięć.
    a.speed = velocity_[axis - 1]; a.speed.valid = a.speed.fresh(now, 250ms) && !bus_.fault_;
    if (jog_stopped(axis, now)) a.speed = {0, d.position.stamp, !bus_.fault_};
    a.can_jog = !result.busy && !result.fault && ready(axis, now) && jog_stopped(axis, now);
    a.can_zero = !result.busy && a.available && stopped(axis, now, 100ms);
    if (a.speed.valid && d.motor_state.fresh(now, 150ms)) {
      switch (d.motor_state.value) {
        case 1: a.motion = jog_stopped(axis, now) ? "zatrzymany" : "stan nieznany"; break;
        case 2: a.motion = "rozpędza się"; break;
        case 3: a.motion = "hamuje"; break;
        case 4: a.motion = "w prędkości"; break;
        default: break;
      }
    }
  }
  return result;
}
}  // namespace mks_servo_can
