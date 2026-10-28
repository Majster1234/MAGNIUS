#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/motion_sample.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include "mks_servo_can/diagnostic_message.hpp"
#include "mks_servo_can/trace.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <linux/can/error.h>

namespace mks_servo_can {
namespace {
constexpr double kPi = 3.14159265358979323846;
double milliseconds(Time a, Time b) { return std::chrono::duration<double, std::milli>(a - b).count(); }
// Upper bound for standard classic CAN including stuffing/inter-frame spacing.
double bits(std::size_t dlc) { return can_frame_bits(dlc); }
bool program_query_window(Time now,Time started) {
  // MKS odpowiada z tym samym CAN ID co host. Pozostawiamy czas na odpowiedź
  // przed następnym F5, bez uzależniania ruchu od otrzymania pomiaru.
  return started!=Time{} && now>=started &&
      (now-started)%kStreamingCycle<kStreamingCycle-std::chrono::milliseconds(4);
}
}
void Aggregate::add(double value) {
  if (!std::isfinite(value)) return;
  ++count; const auto delta = value - mean; mean += delta / count;
  m2 += delta * (value - mean);
  minimum = count == 1 ? value : std::min(minimum, value);
  maximum = count == 1 ? value : std::max(maximum, value);
}
double Aggregate::deviation() const { return count > 1 ? std::sqrt(std::max(0.0, m2 / (count - 1))) : 0; }

CanBusManager::CanBusManager(BusConfig config, std::unique_ptr<Transport> transport, TimeSource clock)
    : config_(std::move(config)), clock_(std::move(clock)), transport_(std::move(transport)), manual_(std::make_unique<ManualMotorController>(*this)) {
  if (!transport_ || !config_.manual_max_rpm || config_.manual_max_rpm > 3000 || (config_.bitrate != 500000 && config_.bitrate != 1000000) || config_.cycle != kStreamingCycle || config_.use_ack ||
      config_.rx_poll_timeout_ms<1 || config_.rx_poll_timeout_ms>1000 ||
      config_.response_timeout.count() < 1 || config_.feedback_timeout <= kEncoderPeriod ||
      config_.command_timeout < config_.cycle || config_.source_lease_timeout < config_.cycle || !std::isfinite(config_.utilization_limit) ||
      config_.utilization_limit <= 0 || config_.utilization_limit > 0.5)
    throw std::invalid_argument("Nieprawidłowy budżet/czas konfiguracji CAN");
  std::array<bool, 2048> addresses{};
  for (const auto &axis : config_.axes) {
    if (!config_.motion_enabled && axis.address == 0) continue;
    if (axis.address == 0 || axis.address > kMaximumAddress || addresses[axis.address] ||
        !std::isfinite(axis.gear_ratio) || axis.gear_ratio <= 0 ||
        (axis.direction != 1 && axis.direction != -1) || !std::isfinite(axis.offset) ||
        !std::isfinite(axis.lower) || !std::isfinite(axis.upper) || axis.lower >= axis.upper ||
        !std::isfinite(axis.max_velocity) || axis.max_velocity <= 0 ||
        !std::isfinite(axis.max_acceleration) || axis.max_acceleration <= 0 ||
        !std::isfinite(axis.tracking_error) || axis.tracking_error <= 0 || axis.acceleration_code == 0)
      throw std::invalid_argument("Niepełna konfiguracja osi: adres, przekładnia, limity, F5 acc");
    joint_to_position(axis, axis.lower); joint_to_position(axis, axis.upper);
    const auto rpm = axis.max_velocity * axis.gear_ratio * 60 / (2 * kPi);
    if (rpm < 1 || rpm > 3000) throw std::invalid_argument("Prędkość osi poza zakresem MKS 1–3000 RPM");
    addresses[axis.address] = true;
  }
  const auto critical_cost = kAxes * (bits(8) + bits(2) + bits(8));
  const auto budget = config_.bitrate * config_.cycle.count() / 1000.0 * std::min(kPlannedUtilization, config_.utilization_limit);
  if (config_.motion_enabled && (critical_cost > budget || streaming_bus_utilization(kAxes,config_.bitrate)>std::min(kPlannedUtilization,config_.utilization_limit)+1e-12))
    throw std::invalid_argument("Streaming 100 Hz sześciu osi wymaga budżetu 40% i magistrali 1 Mbit/s");
  transport_->set_receive_poll_timeout(config_.rx_poll_timeout_ms);
  for(auto &payload:stream_payloads_) payload.resize(8);
  rebuild_axis_lookup();

  for(unsigned axis=1;axis<=kAxes;++axis) shaft_reference_changed(axis);

}
void CanBusManager::shaft_reference_changed(unsigned axis) {
  if(axis<1 || axis>6) return;
  const auto next=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  shaft_segments_[axis-1]=std::max(shaft_segments_[axis-1]+1,static_cast<std::uint64_t>(next));
  shaft_flags_[axis-1]|=1;
}
void CanBusManager::record_shaft_position(unsigned axis,std::int64_t ticks,Time stamp,std::int64_t kernel_utc_ns) {
  if(axis<1 || axis>6) return;
  if(kernel_utc_ns<=0) kernel_utc_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  const auto j=axis-1;
  ShaftPositionSample sample{++shaft_sequences_[j],shaft_segments_[j],static_cast<std::uint64_t>(trace_ns(stamp)),
    static_cast<std::uint64_t>(kernel_utc_ns),ticks,shaft_flags_[j],static_cast<std::uint8_t>(axis)};
  if(shaft_trace_.push(sample)) shaft_flags_[j]=0;
}
Time CanBusManager::current_time(Time fallback) const {
  return clock_ ? std::max(fallback, clock_()) : fallback;
}
void CanBusManager::rebuild_axis_lookup() {
  axis_lookup_.fill(kAxes);
  for(unsigned j=0;j<kAxes;++j) if(config_.axes[j].address) axis_lookup_[config_.axes[j].address]=j;
}
bool CanBusManager::phased_program() const {
  return active_ && source_==Source::program && executor_.phase_origin!=Time{};
}
unsigned CanBusManager::feedback_group() const {
  return phased_program() && cycle_start_>=executor_.phase_origin ?
      ((cycle_start_-executor_.phase_origin)/kMotionCycle)%2 : cycle_number_%2;
}
double CanBusManager::stream_cycle_bits(unsigned participants) const {
  return participants*(bits(8)+bits(2)+bits(8));
}
double CanBusManager::mandatory_cycle_bits() const {
  unsigned participants=0;
  if(active_) for(unsigned j=0;j<kAxes;++j) participants+=source_!=Source::program || executor_.status.participating[j];
  else participants=known_count_;
  if(active_) return stream_cycle_bits(participants);
  return ((participants+1)/2)*(bits(2)+bits(8));
}
void CanBusManager::send_stream_payload(unsigned joint,const std::vector<std::uint8_t> &payload) {
  transport_->send(config_.axes[joint].address,payload);++stats_.tx;
  trace_note("stream_point","bez ACK",config_.axes[joint].address,0xf5);
}
Sample<double> CanBusManager::streaming_feedback_position(unsigned joint,Time now) const {
  const auto &a=config_.axes[joint];const auto cached=transport_->position_feedback(a.address);
  if(cached.valid) {
    const Time stamp{std::chrono::nanoseconds(cached.timestamp_ns)};
    if(now>=stamp && now-stamp<kProgramFeedbackTimeout) return {position_to_joint(a,cached.pulses),stamp,true};
  }
  const auto &position=devices_[a.address].position;
  if(position.fresh(now,kProgramFeedbackTimeout)) return {position_to_joint(a,position.value),position.stamp,true};
  return {};
}
CanBusManager::~CanBusManager() = default;
double CanBusManager::position_to_joint(const AxisConfig &axis, std::int64_t ticks) {
  return JointConversion{axis.gear_ratio,axis.direction,axis.offset,axis.address,true}.position(ticks);
}
std::int32_t CanBusManager::joint_to_position(const AxisConfig &axis, double position) {
  const auto ticks = (position - axis.offset) * axis.gear_ratio / axis.direction * 16384 / (2 * kPi);
  if (!std::isfinite(ticks) || ticks < -8388607 || ticks > 8388607) throw std::invalid_argument("Zakres pozycji przekracza F5 int24");
  return static_cast<std::int32_t>(std::llround(ticks));
}
void CanBusManager::open(Time now) {
  try {
    transport_->open(config_.interface);
    if(transport_->bitrate() && transport_->bitrate()!=config_.bitrate) {
      transport_->close();throw std::runtime_error("Bitrate konfiguracji różni się od fizycznego interfejsu CAN");
    }
    next_cycle_=now;cycle_initialized_=false;message_="Magistrala CAN dostępna";
  }
  catch (const std::exception &e) { fault(e.what()); }
}
void CanBusManager::close() {
  if (manual_->busy() && !fault_) {
    try { transport_->send(manual_->axis(), speed_request(manual_->axis(), false, 0)); ++stats_.tx; } catch (...) {}
    fault("Zamknięcie komunikacji podczas sterowania ręcznego");
  }
  if (active_) stop("Zamknięcie komunikacji podczas sterowania");
  transport_->close(); scanning_ = false;
  for (auto &p : pending_) p.used = false;
}
bool CanBusManager::service_allowed() const { return (configuration_stage_ == "boot" || (configuration_ready_ && !configuration_locked_)) && !stats_.stop_pending && !active_ && (!manual_->busy() || fault_); }
bool CanBusManager::start_scan() {
  if (!service_allowed() || scanning_ || fault_ || !transport_->is_open()) return false;
  scanning_ = true; scan_address_ = 1; checked_ = 0; message_ = "Skanowanie adresów 1–20"; return true;
}
void CanBusManager::clear_fault() {
  if (!service_allowed()) return;
  program_owned_=false;
  fault_ = false; stop_sent_ = false; latest_.reset(); last_sequence_ = 0;
  source_ = Source::none; lease_stamp_ = {};
  manual_ = std::make_unique<ManualMotorController>(*this);
  std::array<bool, kAxes> changed; changed.fill(true); manual_->configuration_changed(changed);
  for (auto &p : pending_) p.used = false;
  config_.program_staged_reset_verified=false; // Ponowny odbiór po niepewnym stanie napędów.
  message_ = "Błąd skasowany — wymagana ponowna aktywacja";
}
void CanBusManager::reset_session(ResetMode mode) {
  if (active_ || transport_->is_open()) throw std::logic_error("Reset sesji wymaga zamkniętego transportu");
  for(unsigned axis=1;axis<=kAxes;++axis) shaft_reference_changed(axis);
  executor_={};program_owned_=false;
  scan_after_bootstrap_=mode==ResetMode::clear_discovery;
  if(mode==ResetMode::preserve_discovery) {
    for(unsigned n=0;n<known_count_;++n) {
      auto &device=devices_[known_addresses_[n]];
      const auto identity=device.identity;
      device={};device.detected=true;device.identity=identity;
    }
  } else { devices_={};known_count_=0; }
  pending_ = {}; quarantine_ = {};stop_sequence_={};
  next_position_ = {}; next_enable_ = {}; next_stall_ = {}; next_motor_ = {}; next_mode_ = {}; next_speed_ = {};
  manual_ = std::make_unique<ManualMotorController>(*this);
  latest_.reset(); stats_ = {}; checked_ = 0; cursor_ = 1;
  last_rx_ = {};last_tick_={};last_cycle_wake_={};cycle_number_=0; cycle_initialized_ = false; clear_fault();
}
std::size_t CanBusManager::axis_index(std::uint16_t address) const {
  return address<=kMaximumAddress ? axis_lookup_[address] : kAxes;
}
bool CanBusManager::feedback_good(Time now) const {
  for (const auto &axis : config_.axes) {
    const auto &d = devices_[axis.address];
    if (!d.detected || d.identity.model != Model::kServo42D ||
        !d.position.fresh(now, config_.feedback_timeout) ||
        !d.enable.fresh(now, std::chrono::milliseconds(300)) || !d.enable.value ||
        !d.stall.fresh(now, std::chrono::milliseconds(300)) || d.stall.value ||
        !d.mode.valid || !supports_motor_state(d.mode.value)) return false;
  }
  return true;
}
std::string CanBusManager::feedback_failure_reason(Time now) const {
  std::string message;
  for (unsigned j = 0; j < kAxes; ++j) {
    const auto &axis = config_.axes[j];
    const auto &d = devices_[axis.address];
    std::string reasons;
    const auto add = [&reasons](const std::string &reason) {
      if (!reasons.empty()) reasons += ", ";
      reasons += reason;
    };
    if (!d.detected) add("napęd niewykryty");
    else if (d.identity.model != Model::kServo42D) add("nieobsługiwany model napędu");
    else {
      if (!d.position.fresh(now, config_.feedback_timeout))
        add(sample_diagnostic(d.position, now, config_.feedback_timeout, "pozycja (0x31)"));
      if (!d.enable.fresh(now, std::chrono::milliseconds(300)))
        add(sample_diagnostic(d.enable, now, std::chrono::milliseconds(300), "ENABLE (0x3A)"));
      else if (!d.enable.value) add("ENABLE=0 — napęd wyłączony");
      if (!d.stall.fresh(now, std::chrono::milliseconds(300)))
        add(sample_diagnostic(d.stall, now, std::chrono::milliseconds(300), "stan stall (0x3E)"));
      else if (d.stall.value) add("stall=1 — napęd zgłosił zablokowanie wału");
      if (!d.mode.valid) add("brak potwierdzonego trybu pracy (0x82)");
      else if (!supports_motor_state(d.mode.value))
        add("nieobsługiwany tryb pracy (0x82): " + std::to_string(d.mode.value));
    }
    if (!reasons.empty()) {
      if (!message.empty()) message += "; ";
      message += axis_diagnostic_label(j + 1, axis.address) + ": " + reasons;
    }
  }
  return message;
}
bool CanBusManager::activate(Source source, Time now) {
  if (!configuration_ready_ || configuration_locked_) return false;
  if (config_.source_lease_required && source!=Source::tcp_jog) {
    if (source_ == Source::none || now < lease_stamp_ || now - lease_stamp_ >= config_.source_lease_timeout) return false;
    source = source_;
  }
  if (program_owned_ || executor_.status.busy || source == Source::program) return false;
  if (active_ || manual_->busy() || scanning_ || fault_ || source == Source::none ||
      (!config_.motion_enabled && !(source==Source::tcp_jog && config_.tcp_jog_enabled)) ||
      !transport_->is_open() || !feedback_good(now)) return false;
  for (std::size_t i = 0; i < kAxes; ++i) {
    committed_positions_[i] = position_to_joint(config_.axes[i], devices_[config_.axes[i].address].position.value);
    const auto &d=devices_[config_.axes[i].address];
    if (committed_positions_[i] < config_.axes[i].lower || committed_positions_[i] > config_.axes[i].upper ||
        !d.responses_disabled || !d.synchronization_disabled || !d.microstep.valid || !manual_->calibration_valid(i+1)) return false;
  }
  try {
    for(unsigned j=0;j<kAxes;++j) (void)streaming_setpoint(config_.axes[j],committed_positions_[j],0,devices_[config_.axes[j].address].microstep.value);
  } catch(const std::exception &e) { message_=e.what();return false; }
  active_ = true; source_ = source; last_command_ = now; lease_stamp_ = now; latest_.reset();
  previous_velocity_.fill(0);streaming_states_={}; last_sequence_ = 0; message_ = "Sterowanie aktywne"; return true;
}
bool CanBusManager::submit(const CommandBatch &batch) {
  if(!configuration_ready_ || configuration_locked_ || batch.configuration_generation!=configuration_generation_) return false;
  if (executor_.status.busy || batch.source == Source::program) return false;
  if (!active_ || fault_ || batch.source != source_ || batch.sequence <= last_sequence_) return false;
  if(source_==Source::tcp_jog && batch.control_generation!=control_generation_) return false;
  if (latest_) ++stats_.dropped_commands;
  latest_ = batch; last_sequence_ = batch.sequence; return true;
}
void CanBusManager::renew_source_lease(Source source, Time now) {
  if(source_==Source::tcp_jog || source==Source::tcp_jog) return;
  if (fault_ || program_owned_ || executor_.status.busy) return;
  if (!active_) { source_ = source; lease_stamp_ = now; return; }
  if (source == Source::none) { fault("Źródło sterowania zwolniło uprawnienie ruchu"); return; }
  if (source != source_) {
    latest_.reset(); source_ = source;
  }
  lease_stamp_ = now;
}
void CanBusManager::fault(const std::string &message, bool critical) {
  if (fault_) return;
  trace_note("bus_fault", message);
  trace_freeze_after(std::chrono::seconds(5));
  const bool was_active = active_;
  const bool program_failed=executor_.status.busy || executor_.status.state=="ready";
  if(program_failed) {
    executor_.status.busy=false;executor_.status.state="failed";executor_.status.message=message;
  }
  fault_ = true; active_ = false; scanning_ = false; message_ = message;
  manual_->transport_fault();
  latest_.reset(); source_ = Source::none; tcp_owner_.clear();tcp_last_stop_=last_tick_;++stats_.errors;
  for (auto &p : pending_) p.used = false;
  if (was_active) { config_.program_staged_reset_verified=false;emergency_stop(); }
  // ENABLE=0 jest ostatnim poleceniem dla każdej osi, po wyczyszczeniu celów F7/F6.
  if(critical && profile_.disable_enable_on_critical_fault) disable_enable_after_fault();
  else if(critical) message_+="; reguła awarii krytycznej: automatyczne ENABLE=0 wyłączone w konfiguracji robota";
  if(program_failed) executor_.status.message=message_;
}
void CanBusManager::disable_enable_after_fault() {
  std::string sent, failed;
  // J1–J6 mają adresy CAN 1–6, także przed kalibracją/wykryciem napędów.
  // Ta ścieżka omija kolejkę operacji i nie czeka na ACK ani gotowość osi.
  for(unsigned address=1;address<=kAxes;++address) {
    const auto label=axis_diagnostic_label(address,address);
    try {
      if(!transport_->is_open()) throw std::runtime_error("SocketCAN jest zamknięty");
      transport_->send(address,enable_request(address,false));++stats_.tx;
      if(!sent.empty()) sent+=", ";
      sent+=label;
      trace_note("critical_enable_off_sent","F3 ENABLE=0; bez potwierdzenia",address,0xf3);
    } catch(const std::exception &e) {
      if(!failed.empty()) failed+="; ";
      failed+=label+": "+e.what();
      trace_note("critical_enable_off_failed",e.what(),address,0xf3);
    } catch(...) {
      if(!failed.empty()) failed+="; ";
      failed+=label+": nieznany błąd wysyłania";
      trace_note("critical_enable_off_failed","nieznany błąd wysyłania",address,0xf3);
    }
  }
  if(!sent.empty()) message_+="; reguła awarii krytycznej: wysłano ENABLE=0 do "+sent+"; wyłączenie niepotwierdzone pomiarami";
  if(!failed.empty()) message_+="; nie wysłano ENABLE=0: "+failed;
}
void CanBusManager::stop(const std::string &reason, bool critical) {
  if (manual_->busy() && !fault_) manual_->abort(reason,critical);
  else fault(reason,critical);
}
void CanBusManager::emergency_stop() {
  if(stop_sent_) return;
  stop_sent_=true;stop_sequence_={};auto &stop=stop_sequence_;
  stop.phase=StopSequence::Phase::f7;stop.begun=stop.sent=current_time(last_tick_);
  stats_.stop_pending=false;stats_.stop_f7_acks=stats_.stop_zero_acks=stats_.stop_rejected=stats_.stop_failed=0;
  for(unsigned j=0;j<kAxes;++j) {
    const auto address=config_.axes[j].address;stop.addresses[j]=address;if(!address) continue;
    if(devices_[address].detected) stop.required|=1u<<j;
    try {transport_->send(address,stop_request(address));++stats_.tx;} catch(...) {stats_.stop_failed|=1u<<j;}
  }
  trace_note("stop_f7_all","bez oczekiwania na ACK");
  send_zero_stop(stop.sent);
  stats_.stop_elapsed_ms=milliseconds(current_time(last_tick_),stop.begun);
  stop.phase=StopSequence::Phase::done;stats_.stop_pending=false;
  trace_note("stop_cleanup_done","STOP wysłano bez ACK; nie jest potwierdzeniem postoju");
}
void CanBusManager::send_zero_stop(Time now) {
  auto &stop=stop_sequence_;stop.phase=StopSequence::Phase::zero;stop.replied=0;stop.sent=now;
  (void)now;
  for(unsigned j=0;j<kAxes;++j) if(stop.addresses[j]) {
    try { transport_->send(stop.addresses[j],speed_request(stop.addresses[j],false,0));++stats_.tx; }
    catch(...) { stats_.stop_failed|=1u<<j; }
  }
  trace_note("stop_zero_all");
}
bool CanBusManager::can_send(std::uint16_t address, Time now) const {
  if (now < quarantine_[address]) return false;
  for (const auto &p : pending_) if (p.used && p.address == address) return false;
  return std::count_if(pending_.begin(),pending_.end(),[](const Pending &p){return p.used;})<4;
}
bool CanBusManager::reserve(std::size_t request_length, std::size_t response_length, bool critical, unsigned extra_responses) {
  const auto total=bits(request_length)+(response_length ? (1+extra_responses)*bits(response_length) : 0);
  const auto budget=config_.bitrate*std::chrono::duration<double>(config_.cycle).count()*std::min(kPlannedUtilization,config_.utilization_limit);
  if(spent_bits_+total>budget || (!critical && background_bits_+total>std::max(0.,budget-mandatory_cycle_bits()))) {
    ++stats_.deferred;return false;
  }
  spent_bits_+=total;if(!critical) background_bits_+=total;
  return true;
}
bool CanBusManager::send_query(std::uint16_t address, Kind kind, Time now, bool critical) {
  if (!can_send(address, now)) return false;
  std::uint8_t command{}; std::size_t length{3};
  const std::vector<std::uint8_t> *payload=nullptr;
  switch (kind) {
    case Kind::identity: command = 0x40; length = 7; break;
    case Kind::mode: command = 0x82; break;
    case Kind::response_mode:command=0x8c;length=4;break;
    case Kind::sync_mode:command=0x4a;break;
    case Kind::microstep:command=0x84;break;
    case Kind::position: command = 0x31; length = 8; break;
    case Kind::enable: command = 0x3a; break;
    case Kind::stall: command = 0x3e; break;
    case Kind::motor: command = 0xf1; break;
    case Kind::speed: command = 0x32; length = 4; break;
    case Kind::manual: case Kind::program_setup: return false;
  }
  const bool parameter=kind==Kind::mode || kind==Kind::response_mode || kind==Kind::sync_mode || kind==Kind::microstep;
  query_payload_.resize(parameter ? 3 : 2);
  query_payload_[0]=parameter ? 0 : command;
  query_payload_[1]=parameter ? command : static_cast<std::uint8_t>(address+command);
  if(parameter) query_payload_[2]=static_cast<std::uint8_t>(address+command);
  payload=&query_payload_;
  now=current_time(now);
  if (!reserve(payload->size(), length, critical)) return false;
  transport_->send(address, *payload); ++stats_.tx;
  for (auto &p : pending_) if (!p.used) {
    auto deadline = now + config_.response_timeout;
    p = {true, address, kind, command, length, now, deadline};
    TraceEvent e; e.event = "request"; e.axis = address; e.command = command;
    e.request_ns = trace_ns(now); e.deadline_ns = trace_ns(deadline); trace(e); return true;
  }
  return false;
}
bool CanBusManager::send_manual(std::uint16_t address, const std::vector<std::uint8_t> &data,
    std::size_t response_length, Time now, bool urgent, std::chrono::milliseconds timeout) {
  if (fault_ || !transport_->is_open()) return false;
  if(!data.empty() && (data[0]==0xf5 || data[0]==0xf6 || data[0]==0xf7 || data[0]==0xfe)) {
    if(!urgent && !reserve(data.size(),0,true)) return false;
    transport_->send(address,data);++stats_.tx;trace_note("motion_sent_no_ack","",address,data[0]);return true;
  }
  if (urgent) {
    for (auto &p : pending_) if (p.used && p.address == address) p.used = false;
    quarantine_[address] = {};
    if (std::all_of(pending_.begin(), pending_.end(), [](const auto &p) { return p.used; })) pending_.back().used = false;
  }
  if (!can_send(address, now)) return false;
  if (!reserve(data.size(), response_length, true) && !urgent) return false;
  transport_->send(address, data); ++stats_.tx;
  for (auto &p : pending_) if (!p.used) {
    p = {true, address, Kind::manual, data[0] == 0 ? data[1] : data[0], response_length, now,
         now + (timeout.count() > 0 ? timeout : config_.response_timeout)};
    TraceEvent e; e.event = "request"; e.axis = address; e.command = p.command;
    e.flags = 1; e.request_ns = trace_ns(now); e.deadline_ns = trace_ns(p.deadline); trace(e);
    return true;
  }
  return false;
}
void CanBusManager::next_scan(std::uint16_t address) {
  if (!scanning_ || scan_address_ != address) return;
  ++checked_; ++scan_address_;
  if (scan_address_ > kScanMaximumAddress) { scanning_ = false; message_ = "Skanowanie zakończone"; }
}
bool CanBusManager::decode(Pending &p, const Frame &frame, Time stamp) {
  auto &d = devices_[p.address];
  switch (p.kind) {
    case Kind::identity: {
      const auto parsed = parse_identity(p.address, frame.data); if (!parsed) return false;
      d.identity = *parsed;
      if(!d.detected && known_count_<known_addresses_.size()) known_addresses_[known_count_++]=p.address;
      d.detected = true; next_mode_[p.address] = Time{}; next_scan(p.address); break;
    }
    case Kind::mode: { const auto v = parse_work_mode(p.address, frame.data); if (!v) return false; d.mode.set(*v, stamp); break; }
    case Kind::response_mode:
      if(frame.data[1]>1 || frame.data[2]>1) return false;
      d.responses_known=true;d.responses_disabled=frame.data[1]==0 && frame.data[2]==0;break;
    case Kind::sync_mode:
      if(frame.data[1]>1) return false;
      d.synchronization_known=true;d.synchronization_disabled=frame.data[1]==0;break;
    case Kind::microstep:
      try {speed_scale(frame.data[1]);} catch(...) {return false;}
      d.microstep.set(frame.data[1],stamp);break;
    case Kind::position: {
      const auto v = parse_position(p.address, frame.data); if (!v) return false;
      d.position.set(*v, stamp); d.position_sent = p.sent;
      record_shaft_position(p.address,*v,stamp,frame.kernel_utc_ns);
      if (d.consecutive_timeouts >= 3) next_position_[p.address] = next_cycle_;
      break;
    }
    case Kind::enable: { const auto v = parse_enable(p.address, frame.data); if (!v) return false; d.enable.set(*v, stamp); break; }
    case Kind::stall: { const auto v = parse_stall(p.address, frame.data); if (!v) return false; d.stall.set(*v, stamp); break; }
    case Kind::motor: { const auto v = parse_motor_state(p.address, frame.data); if (!v) return false; d.motor_state.set(*v, stamp); break; }
    case Kind::speed: { const auto v = parse_speed(p.address, frame.data); if (!v) return false; d.speed.set(*v, stamp); break; }
    case Kind::manual:
      manual_->reply(frame.data, stamp);
      if(p.command==0x31 && d.position.valid && d.position.stamp==stamp)
        record_shaft_position(p.address,d.position.value,stamp,frame.kernel_utc_ns);
      break;
    case Kind::program_setup: program_setup_reply(frame, stamp); break;

  }
  const auto rtt = milliseconds(stamp, p.sent);
  d.last_rtt_ms = rtt; d.rtt.add(rtt); stats_.rtt.add(rtt); d.last_ok = stamp; d.consecutive_timeouts = 0;
  return true;
}
void CanBusManager::receive(Time now) {
  for (unsigned i = 0; i < 64; ++i) {
    auto frame = transport_->receive(); if (!frame) break;
    const auto stamp = frame->received_at == Time{} ? now : frame->received_at;
    if (frame->error) {
      ++stats_.errors;
      if (frame->error_flags & CAN_ERR_BUSOFF) { fault("BUS-OFF — wymagany reset i ponowna aktywacja"); break; }
      continue;
    }
    ++stats_.rx;
    if (last_rx_ != Time{} && stamp >= last_rx_) stats_.rx_interval_ms.add(milliseconds(stamp, last_rx_));
    last_rx_ = stamp;
    if (frame->address == 0 || frame->address > kMaximumAddress) { ++stats_.invalid; continue; }
    if(frame->data.size()==3 && frame->data[0]==0xf5 &&
        validate_response(frame->address,frame->data,0xf5,3).valid) {
      if(frame->data[1]==2) { ++stats_.completion_reports;trace_note("previous_move_complete","",frame->address,0xf5);continue; }
      if(frame->data[1]==3 || frame->data[1]==0) {
        ++stats_.endstop_reports;
        if(active_ && axis_index(frame->address)<kAxes)
          fault(axis_diagnostic_label(axis_index(frame->address)+1,frame->address)+
              ": raport błędu/krańcówki F5, status="+std::to_string(frame->data[1]));
        continue;
      }
    }
    if(manual_->group_reply(frame->address,frame->data,stamp)) continue;
    auto matched = false;
    for (auto &p : pending_) {
      if (!p.used || p.address != frame->address || frame->data.empty() || p.command != frame->data.front()) continue;
      matched = true;
      TraceEvent e; e.axis = p.address; e.command = p.command; e.request_ns = trace_ns(p.sent);
      e.deadline_ns = trace_ns(p.deadline); e.duration_ns = trace_ns(stamp) - trace_ns(p.sent);
      e.kernel_utc_ns = frame->kernel_utc_ns;
      if (stamp < p.sent || stamp >= p.deadline) { e.event = "reply_late"; trace(e); ++stats_.late; break; }
      if (!validate_response(p.address, frame->data, p.command, p.length).valid || !decode(p, *frame, stamp)) {
        e.event = "reply_invalid"; trace(e);
        ++stats_.invalid; ++devices_[frame->address].invalid; break;
      }
      e.event = "reply_ok"; trace(e);
      p.used = false; break;
    }
    if (!matched) { ++stats_.late; trace_note("reply_unmatched", "", frame->address, frame->data.empty() ? 0 : frame->data[0]); }
  }
}
void CanBusManager::expire(Time now) {
  for (auto &p : pending_) {
    if (!p.used || now < p.deadline) continue;
    p.used = false; ++stats_.timeouts;
    TraceEvent e; e.event = "request_timeout"; e.axis = p.address; e.command = p.command;
    e.request_ns = trace_ns(p.sent); e.deadline_ns = trace_ns(p.deadline);
    e.duration_ns = trace_ns(now) - trace_ns(p.sent); e.flags = p.kind == Kind::manual; trace(e);
    quarantine_[p.address] = now + config_.response_timeout;
    if (p.kind == Kind::identity) next_scan(p.address);
    else {
      auto &d = devices_[p.address]; ++d.timeouts; ++d.consecutive_timeouts;
      if (d.consecutive_timeouts >= 3) {
        d.mode.valid = d.enable.valid = d.stall.valid = d.motor_state.valid = false;d.responses_known=d.synchronization_known=false;d.microstep.valid=false;
      }
    }
    if(active_ && source_==Source::program && p.kind!=Kind::program_setup) {
      // Brak odczytu wstrzymuje grupę przy kontroli; nie przerywa ruchu.
      if(p.kind==Kind::position) devices_[p.address].position.valid=false;
      continue;
    }
    if (p.kind == Kind::program_setup) {
      fault(axis_diagnostic_label(axis_index(p.address)+1,p.address)+
          ": brak odpowiedzi podczas uzbrajania programu; kod komendy="+std::to_string(p.command)+
          ", czas oczekiwania="+diagnostic_number(milliseconds(now,p.sent))+" ms"); break;
    }
    if (p.kind == Kind::manual) { manual_->timeout(); break; }
  }
}
void CanBusManager::start_cycle(Time now) {
  if(cycle_initialized_) {
    stats_.cycle_ms.add(milliseconds(now,last_cycle_wake_));
    const auto skipped=(now-next_cycle_)/config_.cycle;
    stats_.missed_cycles+=skipped;
    if(active_ && !feedback_recorded_) ++stats_.missed_cycles;
    cycle_number_+=skipped+1;next_cycle_+=config_.cycle*(skipped+1);
    stats_.cycle_bits=spent_bits_;
    stats_.scheduled_utilization=spent_bits_/(config_.bitrate*std::chrono::duration<double>(config_.cycle).count());
    stats_.peak_scheduled_utilization=std::max(stats_.peak_scheduled_utilization,stats_.scheduled_utilization);
  } else { next_cycle_=now+config_.cycle;cycle_number_=0; }
  cycle_initialized_=true;last_cycle_wake_=now;cycle_start_=next_cycle_-config_.cycle;
  spent_bits_=background_bits_=0;diagnostic_transactions_=0;
  motion_this_cycle_=false;feedback_recorded_=false;
}
void CanBusManager::schedule_feedback(Time now) {
  if(active_) {
    if((source_==Source::program || source_==Source::tcp_jog) &&
        !program_query_window(now,source_==Source::tcp_jog ? tcp_started_ : executor_.started)) return;
    for(unsigned j=0;j<kAxes;++j) {
      if(source_==Source::program && !executor_.status.participating[j]) continue;
      const auto address=config_.axes[j].address;
      if(now>=next_position_[address] && send_query(address,Kind::position,now,true))
        next_position_[address]=now+kStreamingCycle;
    }
    return;
  }
  unsigned rank=0;
  for(unsigned n=0;n<known_count_;++n) {
    const auto address=known_addresses_[n];const auto &d=devices_[address];
    if(d.identity.model!=Model::kServo42D) continue;
    const auto axis=axis_index(address);
    if(active_ && (axis==kAxes || (source_==Source::program && !executor_.status.participating[axis]))) continue;
    const bool group=(rank++%2)==feedback_group();
    const bool warming=source_==Source::program && executor_.status.state=="warming";
    if((!group && !warming) || now<next_position_[address]) continue;
    if(send_query(address,Kind::position,now,true))
      next_position_[address]=d.consecutive_timeouts>=3 ? now+std::chrono::milliseconds(1000) :
          (warming ? now+kEncoderPeriod : cycle_start_+kEncoderPeriod);
  }
}
void CanBusManager::schedule_background(Time now) {
  if(active_ && (source_==Source::program || source_==Source::tcp_jog)) {
    if(!program_query_window(now,source_==Source::tcp_jog ? tcp_started_ : executor_.started)) return;
    while(diagnostic_transactions_<kDiagnosticTransactionsPerCycle) {
      Time oldest=now+std::chrono::hours(1);Time *due=nullptr;
      std::uint16_t selected{};Kind selected_kind{};std::chrono::milliseconds period{};
      for(unsigned j=0;j<kAxes;++j) if(source_==Source::tcp_jog || executor_.status.participating[j]) {
        const auto address=config_.axes[j].address;
        if(!can_send(address,now)) continue;
        auto consider=[&](Kind kind,Time &next,std::chrono::milliseconds interval) {
          if(next<=now && next<oldest) { oldest=next;due=&next;selected=address;selected_kind=kind;period=interval; }
        };
        consider(Kind::enable,next_enable_[address],std::chrono::milliseconds(100));
        consider(Kind::stall,next_stall_[address],std::chrono::milliseconds(100));
        consider(Kind::speed,next_speed_[address],std::chrono::milliseconds(50));
        consider(Kind::motor,next_motor_[address],std::chrono::milliseconds(50));
      }
      if(!due || !send_query(selected,selected_kind,now,false)) break;
      // Bez nadrabiania zaległych odczytów po długim oknie komend.
      *due=now+period;++diagnostic_transactions_;
    }
    return;
  }
  if(source_==Source::program && executor_.status.state=="warming") return;
  for(unsigned n=0;n<known_count_;++n) {
    const auto address=known_addresses_[(cursor_+n)%known_count_];auto &d=devices_[address];
    const auto axis=axis_index(address);
    if(d.identity.model!=Model::kServo42D || !can_send(address,now) || d.consecutive_timeouts>=3) continue;
    if(active_ && (axis==kAxes || (source_==Source::program && !executor_.status.participating[axis]))) continue;
    auto try_read=[&](Kind kind,Time &next,std::chrono::milliseconds period) {
      if(now>=next && send_query(address,kind,now,false)) { next=now+period;return true; }return false;
    };
    if(!d.mode.valid) next_mode_[address]={};
    if(!active_ && !d.responses_known) { if(send_query(address,Kind::response_mode,now,false)) continue; }
    if(!active_ && !d.synchronization_known) { if(send_query(address,Kind::sync_mode,now,false)) continue; }
    if(!active_ && !d.microstep.valid) { if(send_query(address,Kind::microstep,now,false)) continue; }
    if((!d.mode.valid || (!active_ && now>=next_mode_[address])) && try_read(Kind::mode,next_mode_[address],std::chrono::milliseconds(5000))) continue;
    if(try_read(Kind::enable,next_enable_[address],std::chrono::milliseconds(100))) continue;
    if(try_read(Kind::stall,next_stall_[address],std::chrono::milliseconds(100))) continue;
    const auto period=(source_==Source::program || manual_->fast_feedback(address)) ? std::chrono::milliseconds(50) : std::chrono::milliseconds(500);
    if((source_==Source::program || manual_->fast_feedback(address)) && try_read(Kind::speed,next_speed_[address],period)) continue;
    if(d.mode.valid && supports_motor_state(d.mode.value)) try_read(Kind::motor,next_motor_[address],period);
  }
  if(known_count_) cursor_=(cursor_+1)%known_count_;
  if(scanning_ && can_send(scan_address_,now)) {
    const bool pending=std::any_of(pending_.begin(),pending_.end(),[](const Pending &p){return p.used && p.kind==Kind::identity;});
    if(!pending) send_query(scan_address_,Kind::identity,now,false);
  }
}
void CanBusManager::schedule_motion(Time now) {
  const bool tcp=source_==Source::tcp_jog;
  if(tcp && !tcp_braking_ && (now<tcp_lease_ || now-tcp_lease_>=std::chrono::milliseconds(200))) {fault("JOG TCP: brak podtrzymania KEEPALIVE; wiek "+diagnostic_number(milliseconds(now,tcp_lease_))+" ms, limit < 200 ms (cały robot)");return;}
  if(!tcp && config_.source_lease_required && (now<lease_stamp_ || now-lease_stamp_>=config_.source_lease_timeout)) {fault("Utrata źródła trajektorii; wiek podtrzymania "+diagnostic_number(milliseconds(now,lease_stamp_))+" ms, limit < "+std::to_string(config_.source_lease_timeout.count())+" ms (cały robot)");return;}
  if(now-last_command_>=config_.command_timeout) {fault("Brak świeżej komendy generatora; wiek "+diagnostic_number(milliseconds(now,last_command_))+" ms, limit < "+std::to_string(config_.command_timeout.count())+" ms (cały robot)");return;}
  if(!feedback_good(now)) {fault("Zatrzymanie sterowania: "+feedback_failure_reason(now));return;}
  if(!latest_ || motion_this_cycle_) return;
  auto batch=*latest_;latest_.reset();
  if(batch.created>now || now>=batch.expires || now-batch.created>=config_.command_timeout) {++stats_.dropped_commands;return;}
  if(!batch.velocities_valid) {fault("Streaming wymaga pozycji i prędkości");return;}
  const double cost=kAxes*bits(8),budget=config_.bitrate*std::chrono::duration<double>(config_.cycle).count()*std::min(kPlannedUtilization,config_.utilization_limit);
  if(spent_bits_+cost>budget) {++stats_.deferred;latest_=batch;return;}
  auto next_streaming=streaming_states_;
  for(unsigned j=0;j<kAxes;++j) {
    const auto &axis=config_.axes[j];
    const auto label=axis_diagnostic_label(j+1,axis.address);
    if(!std::isfinite(batch.velocities[j]) || std::abs(batch.velocities[j])>axis.max_velocity+1e-6) {fault(label+": prędkość referencji "+diagnostic_number(batch.velocities[j]*180/kPi)+" °/s poza limitem ±"+diagnostic_number(axis.max_velocity*180/kPi)+" °/s");return;}
    if(tcp && (!std::isfinite(batch.accelerations[j]) || std::abs(batch.accelerations[j])>axis.max_acceleration+1e-6 ||
        !std::isfinite(batch.remaining_time_sec) || batch.remaining_time_sec<0)) {fault(label+": nieprawidłowe hamowanie TCP; przyspieszenie="+diagnostic_number(batch.accelerations[j]*180/kPi)+" °/s², limit ±"+diagnostic_number(axis.max_acceleration*180/kPi)+" °/s², czas do zatrzymania="+diagnostic_number(batch.remaining_time_sec)+" s");return;}
    const auto feedback=streaming_feedback_position(j,now);
    if(tcp && feedback.valid) {
      if(std::abs(batch.positions[j]-feedback.value)>1.25*kPi/180) {
        fault(label+": uchyb śledzenia JOG TCP="+diagnostic_number((batch.positions[j]-feedback.value)*180/kPi)+"°, limit |uchyb| ≤ 1.25°; referencja="+diagnostic_number(batch.positions[j]*180/kPi)+"°, pomiar="+diagnostic_number(feedback.value*180/kPi)+"°");return;
      }
      const double quantization=kPi/(axis.streaming.encoder_cpr*axis.gear_ratio);
      if(!std::isfinite(feedback.value) || feedback.value<axis.lower-quantization-1e-9 || feedback.value>axis.upper+quantization+1e-9) {
        fault(label+": pomiar TCP="+diagnostic_number(feedback.value*180/kPi)+"° poza limitem mechanicznym ["+diagnostic_number(axis.lower*180/kPi)+", "+diagnostic_number(axis.upper*180/kPi)+"]°");return;
      }
    }
    const auto sample=tcp ? tcp_streaming_setpoint(axis,batch.positions[j],committed_positions_[j],next_streaming[j],
        feedback.valid ? std::optional<double>(feedback.value) : std::nullopt,devices_[axis.address].microstep.value,
        batch.accelerations[j],batch.remaining_time_sec,feedback.valid ? std::chrono::duration<double>(now-feedback.stamp).count() : 0.,batch.lookahead_positions[j]) :
        streaming_setpoint(axis,batch.positions[j],batch.velocities[j],next_streaming[j],
        feedback.valid ? std::optional<double>(feedback.value) : std::nullopt,devices_[axis.address].microstep.value,
        tcp ? batch.accelerations[j] : 0.,tcp ? batch.remaining_time_sec : axis.streaming.lookahead_time_sec,
        feedback.valid ? std::chrono::duration<double>(now-feedback.stamp).count() : 0.,
        tcp ? std::optional<double>(batch.lookahead_positions[j]) : std::nullopt);
    std::copy(sample.bytes.begin(),sample.bytes.end(),stream_payloads_[j].begin());
  }
  spent_bits_+=cost;
  for(unsigned j=0;j<kAxes;++j) send_stream_payload(j,stream_payloads_[j]);
  streaming_states_=next_streaming;
  committed_positions_=batch.positions;previous_velocity_=batch.velocities;last_command_=batch.created;
  motion_this_cycle_=true;++stats_.committed;
}
void CanBusManager::tick(Time now) {
  last_tick_=current_time(now);
  if(!transport_->is_open()) return;
  if(fault_) return;
  try {
    if (!cycle_initialized_ || now >= next_cycle_) start_cycle(now);
    receive(now);
    // SocketCAN znakuje ramki w receive(), już po pobraniu czasu na wejściu
    // do tick(). Świeży pomiar nie może wyglądać na pochodzący z przyszłości.
    now = current_time(std::max(now, last_rx_));last_tick_=now;
    if(now>=next_cycle_) start_cycle(now);
    expire(now);
    if (fault_) return;
    manual_->tick(now);
    if (fault_) return;
    if (active_) {
      const bool program_iteration=source_==Source::program;
      if(program_iteration) schedule_program(now); else schedule_motion(now);
      if (fault_) return;
    }
    schedule_feedback(now);
    if (!feedback_recorded_ && source_!=Source::program) {
      Time first = now, last{}; bool complete = true; unsigned participants = 0;
      unsigned rank=0;
      for (unsigned n=0;n<known_count_;++n) {
        const auto address=known_addresses_[n];const auto &d=devices_[address];
        if(d.identity.model!=Model::kServo42D) continue;
        const auto axis=axis_index(address);
        if(active_ && (axis==kAxes || (source_==Source::program && !executor_.status.participating[axis]))) continue;
        if(rank++%2!=feedback_group()) continue;
        ++participants;
        const auto &sample = d.position;
        if (!sample.valid || sample.stamp < cycle_start_ || d.position_sent < cycle_start_) { complete = false; break; }
        first = std::min(first, sample.stamp); last = std::max(last, sample.stamp);
      }
      if (complete && participants) {
        stats_.feedback_span_ms.add(milliseconds(last, first));
        stats_.feedback_round_ms.add(milliseconds(last, cycle_start_)); feedback_recorded_ = true;
      }
    }
    schedule_background(now);
  } catch (const std::exception &e) { fault(std::string("Błąd transportu CAN: ") + e.what()); }
}
BusSnapshot CanBusManager::snapshot() const {
  return {devices_, stats_, transport_->is_open(), scanning_, active_ || manual_->busy(), fault_, checked_, source_, message_};
}
ControlState CanBusManager::control_state() const {
  ControlState state; state.active = active_; state.fault = fault_; state.source = source_;state.program_owned=program_owned_;
  state.configuration_generation=configuration_generation_;
  state.control_generation=control_generation_;
  state.configuration_ready=configuration_ready_ && !configuration_locked_;
  for (std::size_t i = 0; i < kAxes; ++i) {
    const auto &axis = config_.axes[i]; const auto &p = devices_[axis.address].position;
    const bool calibrated=manual_->calibration_valid(i+1);
    state.conversions[i]={axis.gear_ratio,axis.direction,axis.offset,axis.address,calibrated};
    state.positions[i] = {axis.address ? position_to_joint(axis, p.value) : 0, p.stamp, p.valid && calibrated && state.configuration_ready};
  }
  return state;
}
}  // namespace mks_servo_can
