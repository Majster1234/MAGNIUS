#include "robot_can_diagnostics/diagnostics.hpp"
#include "robot_can_diagnostics/preview.hpp"
#include "mks_servo_can/trace.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <spawn.h>
#include <signal.h>
#include <sstream>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

extern char **environ;
using namespace std::chrono_literals;
namespace robot_can_diagnostics {
namespace {
std::string quote(const std::string &value) {
  std::ostringstream text; text << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') text << '\\' << c;
    else if (c < 32) text << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
    else text << c;
  }
  return text.str() + '"';
}
std::string number(double value) { return std::isfinite(value) ? std::to_string(value) : "null"; }
diagnostic_msgs::msg::KeyValue kv(const std::string &key, const std::string &value) {
  diagnostic_msgs::msg::KeyValue result; result.key = key; result.value = value; return result;
}
diagnostic_msgs::msg::DiagnosticStatus status(const std::string &name, int level,
    const std::string &message, std::vector<diagnostic_msgs::msg::KeyValue> values) {
  diagnostic_msgs::msg::DiagnosticStatus result; result.name = name; result.level = level;
  result.message = message; result.hardware_id = "robot-can"; result.values = std::move(values); return result;
}
int restart_service() {
  pid_t child{};
  const char *const arguments[] = {"sudo", "-n", "/usr/bin/systemctl", "restart", "robot-can@can0.service", nullptr};
  if (posix_spawn(&child, "/usr/bin/sudo", nullptr, nullptr, const_cast<char **>(arguments), environ)) return -1;
  for (int i = 0; i < 50; ++i) {
    int exit{};
    if (waitpid(child, &exit, WNOHANG) == child) return WIFEXITED(exit) ? WEXITSTATUS(exit) : -1;
    std::this_thread::sleep_for(100ms);
  }
  kill(child, SIGKILL); int exit{}; waitpid(child, &exit, 0); return -1;
}
template<class T> std::string age(const mks_servo_can::Sample<T> &s, mks_servo_can::Time now) {
  return s.valid ? number(std::chrono::duration<double>(now - s.stamp).count()) : "null";
}
std::string timestamp(mks_servo_can::Time stamp, mks_servo_can::Time monotonic_now) {
  const auto wall = std::chrono::system_clock::now() - (monotonic_now - stamp);
  auto time = std::chrono::system_clock::to_time_t(wall); std::tm utc{}; gmtime_r(&time, &utc);
  std::ostringstream text; text << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ"); return quote(text.str());
}
}
DiagnosticsNode::DiagnosticsNode(std::shared_ptr<mks_servo_can::BusRuntime> runtime,
                               const mks_servo_can::BusConfig &config, std::function<int()> restart_action)
    : Node("robot_can_diagnostics"), runtime_(std::move(runtime)), config_(config), restart_action_(std::move(restart_action)) {
  if(!restart_action_) restart_action_=restart_service;
  program_api_=std::make_shared<ProgramApi>(this,runtime_);
  tcp_jog_api_=std::make_shared<TcpJogApi>(this,runtime_);
  runtime_->require_configuration();
  using Configure = robot_control_interfaces::srv::ConfigureRobot;
  configuration_ = create_service<Configure>("/robot/configuration/control",
      [this](std::shared_ptr<Configure::Request> req, std::shared_ptr<Configure::Response> res) {
        if (resetting_ || req->phase > Configure::Request::BOOTSTRAP) { res->message = "Reset CAN lub błędna faza"; return; }
        const auto issued = req->issued_monotonic_ns;
        const auto now = static_cast<std::uint64_t>(mks_servo_can::trace_ns(mks_servo_can::Clock::now()));
        if (req->phase != Configure::Request::STATUS && (!issued || issued > now || now - issued > 3000000000ULL)) {
          res->message = "Przeterminowane żądanie konfiguracji"; return;
        }
        mks_servo_can::RobotProfile profile; profile.revision = req->configuration.revision;
        profile.changed = req->configuration.changed;
        profile.calibration_invalid = req->configuration.calibration_invalid;
        profile.confirm_calibration = req->confirm_calibration;
        profile.disable_enable_on_critical_fault = req->configuration.disable_enable_on_critical_fault;
        for (std::size_t i = 0; i < 6; ++i) {
          const auto &a = req->configuration.axes[i];
          if (a.axis != i + 1) { res->message = "Wymagane osie 1–6 w stałej kolejności"; return; }
          profile.axes[i] = {a.configured, a.gear_ratio, static_cast<double>(a.direction), a.zero_offset_rad,
              a.lower_rad, a.upper_rad, a.velocity_rad_s, a.acceleration_rad_s2};
        }
        const auto result = runtime_->configure(static_cast<mks_servo_can::ConfigurationPhase>(req->phase),
            req->operation_id, req->expected_revision, profile);
        res->accepted = result.accepted; res->message = result.message; res->revision = result.revision;
        res->operation_id = result.operation_id; res->stage = result.stage; res->locked = result.locked;
        const auto &resolved=result.resolved_configuration;
        res->resolved_configuration.revision=resolved.revision;
        res->resolved_configuration.disable_enable_on_critical_fault=resolved.disable_enable_on_critical_fault;
        res->resolved_configuration.changed=resolved.changed;
        res->resolved_configuration.calibration_invalid=resolved.calibration_invalid;
        for(std::size_t i=0;i<6;++i) {
          const auto &a=resolved.axes[i];auto &out=res->resolved_configuration.axes[i];
          out.axis=i+1;out.configured=a.configured;out.gear_ratio=a.gear_ratio;out.direction=a.direction;
          out.zero_offset_rad=a.offset;out.lower_rad=a.lower;out.upper_rad=a.upper;
          out.velocity_rad_s=a.velocity;out.acceleration_rad_s2=a.acceleration;
        }
      });
  publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/robot/can/diagnostics", rclcpp::QoS(1));
  using Enable = robot_control_interfaces::srv::SetMotorEnable;
  using Zero = robot_control_interfaces::srv::ZeroMotor;
  using Jog = robot_control_interfaces::srv::JogMotor;
  const auto fresh_request = [](std::uint64_t stamp) {
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(mks_servo_can::Clock::now().time_since_epoch()).count();
    return stamp <= static_cast<std::uint64_t>(now) && static_cast<std::uint64_t>(now) - stamp < 100000000;
  };
  enable_ = create_service<Enable>("/robot/manual/enable", [this, fresh_request](std::shared_ptr<Enable::Request> req, std::shared_ptr<Enable::Response> res) {
    res->accepted = !resetting_ && fresh_request(req->issued_monotonic_ns) && runtime_->manual_enable(req->axis, req->enabled);
    res->message = res->accepted ? "Operacja ENABLE przyjęta; wynik w statusie" : "ENABLE odrzucone: brak osi, postój niepotwierdzony lub zajęte sterowanie";
  });
  zero_ = create_service<Zero>("/robot/manual/zero", [this, fresh_request](std::shared_ptr<Zero::Request> req, std::shared_ptr<Zero::Response> res) {
    res->accepted = !resetting_ && fresh_request(req->issued_monotonic_ns) && runtime_->manual_zero(req->axis);
    res->message = res->accepted ? "Zerowanie przyjęte; wynik w statusie" : "Zerowanie wymaga dostępnej osi i potwierdzonego postoju";
  });
  jog_ = create_service<Jog>("/robot/manual/jog", [this, fresh_request](std::shared_ptr<Jog::Request> req, std::shared_ptr<Jog::Response> res) {
    res->accepted = !resetting_ && req->action <= Jog::Request::STOP && fresh_request(req->issued_monotonic_ns) &&
        runtime_->manual_jog(static_cast<mks_servo_can::JogAction>(req->action), req->axis, req->direction,
          req->rpm, req->session, req->sequence, mks_servo_can::Time(std::chrono::nanoseconds(req->issued_monotonic_ns)));
    res->message = res->accepted ? "Komenda JOG przyjęta" : "JOG odrzucony: ENABLE, świeżość danych, limit lub inny właściciel ruchu";
  });
  using JointJog = robot_control_interfaces::srv::JogJoint;
  using EnableAll=robot_control_interfaces::srv::SetAllMotorsEnable;
  enable_all_=create_service<EnableAll>("/robot/manual/enable_all",[this,fresh_request](std::shared_ptr<EnableAll::Request> req,std::shared_ptr<EnableAll::Response> res) {
    res->accepted=!resetting_ && fresh_request(req->issued_monotonic_ns) && runtime_->manual_enable_all(req->enabled);
    res->message=res->accepted ? "Zbiorcze ENABLE przyjęte; wynik potwierdzają stany napędów" : "Zbiorcze ENABLE odrzucone: wymagany postój, świeże pomiary i wolne sterowanie";
  });
  joint_jog_ = create_service<JointJog>("/robot/manual/jog_joint", [this, fresh_request](std::shared_ptr<JointJog::Request> req, std::shared_ptr<JointJog::Response> res) {
    res->accepted = !resetting_ && req->action <= JointJog::Request::STOP && fresh_request(req->issued_monotonic_ns) &&
        runtime_->joint_jog(static_cast<mks_servo_can::JogAction>(req->action), req->axis, req->direction,
            req->velocity_rad_s, req->revision, req->session, req->sequence,
            mks_servo_can::Time(std::chrono::nanoseconds(req->issued_monotonic_ns)));
    res->message = res->accepted ? "JOG przegubu przyjęty" : "JOG przegubu odrzucony: rewizja, profil, limit lub gotowość osi";
  });
  manual_publisher_ = create_publisher<robot_control_interfaces::msg::ManualState>("/robot/manual/state", rclcpp::QoS(1));
  // Podgląd nie może czekać na potwierdzenia wolnego odbiorcy DDS.
  preview_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/robot/preview/joint_states", rclcpp::QoS(6).best_effort());
  manual_timer_ = create_wall_timer(50ms, [this] { publish_manual(); });
  shaft_publisher_=create_publisher<robot_control_interfaces::msg::ShaftTraceBatch>("/robot/diagnostics/shaft_trace",rclcpp::QoS(20).best_effort());
  shaft_timer_=create_wall_timer(50ms,[this]{publish_shaft_trace();});
  if (config.motion_enabled) {
    lease_ = create_subscription<std_msgs::msg::UInt64MultiArray>("/robot/motion/lease", rclcpp::QoS(1).best_effort(),
        [this](std_msgs::msg::UInt64MultiArray::ConstSharedPtr message) {
          if (message->data.size() != 2 || message->data[1] > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) return;
          const auto source = message->data[0] == 1 ? mks_servo_can::Source::trajectory :
              message->data[0] == 2 ? mks_servo_can::Source::jog : mks_servo_can::Source::none;
          const auto created = mks_servo_can::Time(std::chrono::nanoseconds(message->data[1]));
          runtime_->renew_source_lease(source, created);
        });
  }
  stop_ = create_service<std_srvs::srv::Trigger>("/robot/motion/stop",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        runtime_->stop("Żądanie zatrzymania operatora",false);
        response->success = true; response->message = "Zatrzymanie zlecone; wymagany reset przed wznowieniem";
      });
  scan_ = create_service<std_srvs::srv::Trigger>("/robot/can/scan",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = !resetting_ && runtime_->scan();
        response->message = response->success ? "Skanowanie przyjęte" : "SCAN zablokowany: ruch, błąd lub inna operacja";
      });
  reset_ = create_service<std_srvs::srv::Trigger>("/robot/can/reset",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (resetting_ || !runtime_->reset_allowed() || config_.interface != "can0") {
          response->success = false; response->message = "Reset zablokowany podczas sterowania lub dla interfejsu innego niż can0"; return;
        }
        runtime_->shutdown(); runtime_->reset_session(mks_servo_can::ResetMode::preserve_discovery); resetting_ = true; reset_error_.clear();
        reset_future_ = std::async(std::launch::async, restart_action_);
        response->success = true; response->message = "Reset CAN przyjęty";
      });
  timer_ = create_wall_timer(200ms, [this] { publish(); });
}
void DiagnosticsNode::publish_shaft_trace() {
  std::array<mks_servo_can::ShaftPositionSample,256> samples;
  const auto count=runtime_->drain_shaft_trace(samples.data(),samples.size());
  robot_control_interfaces::msg::ShaftTraceBatch batch;
  batch.dropped_total=runtime_->shaft_trace_dropped();batch.samples.reserve(count);
  for(std::size_t i=0;i<count;++i) {
    const auto &s=samples[i];robot_control_interfaces::msg::ShaftPositionSample point;
    point.axis=s.axis;point.sequence=s.sequence;point.segment=s.segment;point.monotonic_ns=s.monotonic_ns;
    point.utc_ns=s.utc_ns;point.ticks=s.ticks;point.flags=s.flags;batch.samples.push_back(point);
  }
  shaft_publisher_->publish(batch);
}
void DiagnosticsNode::publish_manual() {
  const auto begin = mks_servo_can::Clock::now();
  const auto state = runtime_->manual_snapshot(); const auto now = mks_servo_can::Clock::now();
  robot_control_interfaces::msg::ManualState out;
  out.monotonic_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  out.busy = state.busy || resetting_; out.fault = state.fault; out.active_axis = state.axis; out.max_rpm = state.max_rpm;
  out.operation = state.operation; out.message = state.message;
  out.configuration_revision = state.configuration_revision;
  const auto sample_age = [now](const auto &sample) { return sample.stamp == mks_servo_can::Time{} ? -1.0 : std::chrono::duration<double>(now - sample.stamp).count(); };
  for (std::size_t i = 0; i < state.axes.size(); ++i) {
    const auto &a = state.axes[i]; auto &b = out.axes[i]; b.axis = a.axis; b.available = a.available;
    b.prepared = a.prepared; b.enable_known = a.enable.valid; b.enabled = a.enable.value;
    b.position_known = a.position.valid; b.position_degrees = a.position.value; b.position_age_seconds = sample_age(a.position);
    b.speed_known = a.speed.valid; b.speed_rpm = a.speed.value; b.speed_age_seconds = sample_age(a.speed);
    b.enable_age_seconds = sample_age(a.enable); b.can_jog = a.can_jog && !resetting_; b.can_zero = a.can_zero && !resetting_; b.motion = a.motion;
    b.joint_configured = a.configured; b.calibration_valid = a.calibration_valid;
    b.joint_position_known = a.joint_position.valid; b.joint_position_rad = a.joint_position.value;
    b.joint_lower_rad = a.joint_lower; b.joint_upper_rad = a.joint_upper;
    b.jog_min_velocity_rad_s = a.minimum_velocity; b.jog_max_velocity_rad_s = a.maximum_velocity;
    b.configuration_message = a.configuration_message;
    mks_servo_can::TraceEvent e; e.event = "ros_axis"; e.axis = a.axis; e.request_ns = out.monotonic_ns;
    e.position_age_ns = a.position.stamp == mks_servo_can::Time{} ? -1 : mks_servo_can::trace_ns(now) - mks_servo_can::trace_ns(a.position.stamp);
    e.speed_age_ns = a.speed.stamp == mks_servo_can::Time{} ? -1 : mks_servo_can::trace_ns(now) - mks_servo_can::trace_ns(a.speed.stamp);
    e.enable_age_ns = a.enable.stamp == mks_servo_can::Time{} ? -1 : mks_servo_can::trace_ns(now) - mks_servo_can::trace_ns(a.enable.stamp);
    e.flags = unsigned(a.available) | (unsigned(a.position.valid) << 1) | (unsigned(a.enable.valid) << 2) |
        (unsigned(a.enable.value) << 3) | (unsigned(a.can_jog) << 4) | (unsigned(state.fault) << 5);
    mks_servo_can::trace(e);
  }
  mks_servo_can::TraceEvent e; e.event = "ros_publish"; e.request_ns = out.monotonic_ns;
  e.duration_ns = last_manual_publish_ == mks_servo_can::Time{} ? 0 : mks_servo_can::trace_ns(now) - mks_servo_can::trace_ns(last_manual_publish_);
  e.flags = unsigned(out.busy) | (unsigned(out.fault) << 1); e.axis = out.active_axis;
  mks_servo_can::trace(e); last_manual_publish_ = now;
  manual_publisher_->publish(out);
  for (const auto &sample : preview_states(state, now, get_clock()->now(), last_preview_))
    preview_publisher_->publish(sample);
  const auto end = mks_servo_can::Clock::now();
  if (end - begin > 5ms) { e.event = "ros_publish_slow"; e.duration_ns = mks_servo_can::trace_ns(end) - mks_servo_can::trace_ns(begin); mks_servo_can::trace(e); }
}
void DiagnosticsNode::publish() {
  if (resetting_ && reset_future_.wait_for(0ms) == std::future_status::ready) {
    resetting_ = false;
    if (reset_future_.get() == 0) { runtime_->start(false); }
    else reset_error_ = "Brak uprawnienia lub błąd restartu robot-can@can0.service";
  }
  const auto snapshot = runtime_->snapshot(); const auto now = mks_servo_can::Clock::now(); const auto &s = snapshot.statistics;
  diagnostic_msgs::msg::DiagnosticArray message; message.header.stamp = get_clock()->now();
  std::size_t count{}, online{};
  for (const auto &d : snapshot.devices) if (d.detected) { ++count; if (d.last_ok!=mks_servo_can::Time{} && now - d.last_ok < 3s) ++online; }
  const auto bus_state = snapshot.fault ? "BŁĄD" : snapshot.open ? "AKTYWNA" : "NIEDOSTĘPNA";
  message.status.push_back(status("can", snapshot.fault || !snapshot.open ? 2 : 0, snapshot.message, {
      kv("interface", quote(config_.interface)), kv("bitrate", std::to_string(config_.bitrate)), kv("bus_state", quote(bus_state)),
      kv("socket_open", snapshot.open ? "true" : "false"), kv("active", snapshot.active ? "true" : "false"), kv("fault", snapshot.fault ? "true" : "false"),
      kv("device_count", std::to_string(count)), kv("online_count", std::to_string(online)),
      kv("full_round_ms", s.feedback_round_ms.count ? number(s.feedback_round_ms.mean) : "null"),
      kv("cycle_ms", number(s.cycle_ms.mean)), kv("cycle_max_ms", number(s.cycle_ms.maximum)), kv("cycle_jitter_ms", number(s.cycle_ms.deviation())),
      kv("feedback_span_ms", s.feedback_span_ms.count ? number(s.feedback_span_ms.mean) : "null"),
      kv("command_period_ms",std::to_string(config_.cycle.count())),
      kv("motion_use_ack","false"),kv("streaming_lookahead_ms",number(1000*get_parameter("execution_lookahead_time_sec").as_double())),
      kv("encoder_period_ms",std::to_string(snapshot.source==mks_servo_can::Source::program ?
          get_parameter("execution_position_period_ms").as_int() : (snapshot.active ? mks_servo_can::kStreamingCycle.count() : mks_servo_can::kEncoderPeriod.count()))),
      kv("planned_utilization_limit_percent",number(100*std::min(mks_servo_can::kPlannedUtilization,config_.utilization_limit))),
      kv("future_module_reserve_percent",number(100*mks_servo_can::kFutureModuleReservedUtilization)),
      kv("total_bus_planning_limit_percent",number(100*mks_servo_can::kTotalPlannedUtilization)),
      kv("hard_utilization_limit_percent",number(100*config_.utilization_limit)),
      kv("cycle_bits",number(s.cycle_bits)),kv("peak_scheduled_utilization_percent",number(100*s.peak_scheduled_utilization)),
      kv("ack_margin_min_ms",s.ack_margin_ms.count ? number(s.ack_margin_ms.minimum) : "null"),
      kv("sync_lateness_max_ms",s.sync_lateness_ms.count ? number(s.sync_lateness_ms.maximum) : "null"),
      kv("stop_pending",s.stop_pending ? "true" : "false"),kv("stop_f7_ack_mask",std::to_string(s.stop_f7_acks)),
      kv("stop_zero_ack_mask",std::to_string(s.stop_zero_acks)),kv("stop_rejected_mask",std::to_string(s.stop_rejected)),
      kv("stop_failed_mask",std::to_string(s.stop_failed)),kv("stop_elapsed_ms",number(s.stop_elapsed_ms)),
      kv("stop_result",quote(s.stop_pending ? "F7 → F6(0) w toku" : s.stop_failed ? "Niepotwierdzony STOP" : !config_.use_ack && s.stop_elapsed_ms>0 ? "STOP wysłano bez ACK; postój wymaga pomiaru" :
          (s.stop_f7_acks || s.stop_zero_acks) ? "ACK odebrane; kasowanie celu wymaga odbioru" :
          s.stop_elapsed_ms>0 ? "Wysłano STOP; brak potwierdzonych napędów" : "Brak próby")),
      kv("completion_reports",std::to_string(s.completion_reports)),kv("endstop_reports",std::to_string(s.endstop_reports)),
      kv("scheduled_utilization_percent", number(s.scheduled_utilization * 100)), kv("utilization_kind", quote("planowany koszt ramek i odpowiedzi; bez retransmisji")),
      kv("errors", std::to_string(s.errors)), kv("timeouts", std::to_string(s.timeouts)), kv("invalid_responses", std::to_string(s.invalid)), kv("late_responses", std::to_string(s.late)),
      kv("rx_frames", std::to_string(s.rx)), kv("tx_frames", std::to_string(s.tx)), kv("deferred", std::to_string(s.deferred)), kv("missed_cycles", std::to_string(s.missed_cycles)),
      kv("committed_batches", std::to_string(s.committed)), kv("rtt_mean_ms", number(s.rtt.mean)), kv("rtt_max_ms", number(s.rtt.maximum)),
      kv("rx_interval_mean_ms", number(s.rx_interval_ms.mean)), kv("rx_interval_jitter_ms", number(s.rx_interval_ms.deviation())), kv("sampled_at", timestamp(now, now))}));
  const auto error = reset_error_.empty() ? (snapshot.fault ? snapshot.message : "") : reset_error_;
  message.status.push_back(status("scan", error.empty() ? 0 : 2, resetting_ ? "Resetowanie CAN" : snapshot.message, {
      kv("state", quote(resetting_ ? "Resetowanie" : snapshot.scanning ? "Skanowanie" : snapshot.fault ? "Błąd" : "Gotowe")),
      kv("scanning", snapshot.scanning ? "true" : "false"), kv("resetting", resetting_ ? "true" : "false"),
      kv("checked", std::to_string(snapshot.checked)), kv("total", std::to_string(mks_servo_can::kScanMaximumAddress)), kv("error", quote(error))}));
  for (std::size_t address = 1; address < snapshot.devices.size(); ++address) {
    const auto &d = snapshot.devices[address]; if (!d.detected) continue;
    const auto connected = !snapshot.fault && snapshot.open && d.consecutive_timeouts < 3 && d.last_ok!=mks_servo_can::Time{} && now - d.last_ok < 3s;
    const auto enabled = connected && d.enable.fresh(now, 300ms) ? (d.enable.value ? "true" : "false") : "null";
    const auto stall = connected && d.stall.fresh(now, 300ms) ? (d.stall.value ? "true" : "false") : "null";
    const auto motor = connected && d.mode.valid && mks_servo_can::supports_motor_state(d.mode.value) && d.motor_state.fresh(now, 1500ms) ? mks_servo_can::motor_state_name(d.motor_state.value) : "Niedostępny";
    message.status.push_back(status("device/" + std::to_string(address), connected ? 0 : 1, connected ? "Odpowiada" : "Brak odpowiedzi", {
        kv("address", std::to_string(address)), kv("model", quote(mks_servo_can::model_name(d.identity.model))), kv("online", connected ? "true" : "false"),
        kv("enable", enabled), kv("stall", stall), kv("motor_state", quote(motor)), kv("mode", quote(d.mode.valid ? mks_servo_can::work_mode_name(d.mode.value) : "Niedostępny")),
        kv("position_ticks", d.position.valid ? std::to_string(d.position.value) : "null"), kv("position_age_seconds", age(d.position, now)),
        kv("reported_speed_rpm", d.speed.valid ? std::to_string(d.speed.value) : "null"), kv("reported_speed_age_seconds", age(d.speed, now)),
        kv("enable_age_seconds", age(d.enable, now)), kv("stall_age_seconds", age(d.stall, now)), kv("motor_state_age_seconds", age(d.motor_state, now)),
        kv("rtt_ms", number(d.last_rtt_ms)), kv("rtt_min_ms", number(d.rtt.minimum)), kv("rtt_max_ms", number(d.rtt.maximum)), kv("rtt_mean_ms", number(d.rtt.mean)),
        kv("timeouts", std::to_string(d.timeouts)), kv("invalid_responses", std::to_string(d.invalid)),
        kv("last_ok_age_seconds", d.last_ok==mks_servo_can::Time{} ? "null" : number(std::chrono::duration<double>(now - d.last_ok).count())),
        kv("sampled_at", d.last_ok==mks_servo_can::Time{} ? "null" : timestamp(d.last_ok, now))}));
  }
  publisher_->publish(message);
}
}  // namespace robot_can_diagnostics
