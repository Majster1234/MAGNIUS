#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <functional>
#include <limits>
#include "mks_servo_can/protocol.hpp"
#include "mks_servo_can/socketcan.hpp"
#include "mks_servo_can/configuration.hpp"
#include "mks_servo_can/motion_executor.hpp"
#include "mks_servo_can/motion_timing.hpp"
#include "mks_servo_can/shaft_trace.hpp"
#include "mks_servo_can/streaming_motion.hpp"

namespace mks_servo_can {
class ManualMotorController;
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
constexpr std::size_t kAxes = 6;

template<class T> struct Sample {
  T value{};
  Time stamp{};
  bool valid{false};
  bool fresh(Time now, std::chrono::milliseconds age) const {
    return valid && now >= stamp && now - stamp < age;
  }
  void set(T next, Time now) { value = next; stamp = now; valid = true; }
};
struct Aggregate {
  std::uint64_t count{};
  double mean{}, minimum{}, maximum{}, m2{};
  void add(double value);
  double deviation() const;
};
struct DeviceState {
  Identity identity{};
  bool detected{false};
  bool responses_disabled{}, synchronization_disabled{};
  bool responses_known{}, synchronization_known{};
  Sample<std::uint16_t> microstep;
  Time last_ok{};
  Time position_sent{};
  Sample<std::int64_t> position;
  Sample<bool> enable, stall;
  Sample<std::uint8_t> mode, motor_state;
  Sample<std::int16_t> speed;
  Sample<std::uint32_t> watchdog_ms;
  Aggregate rtt;
  double last_rtt_ms{};
  std::uint64_t timeouts{}, invalid{};
  unsigned consecutive_timeouts{};
};
struct AxisConfig {
  std::uint16_t address{};
  double gear_ratio{}, direction{}, offset{}, lower{}, upper{}, max_velocity{}, max_acceleration{}, tracking_error{};
  std::uint8_t acceleration_code{};
  StreamingParameters streaming{};
};
enum class Source { none, trajectory, jog, program, tcp_jog };
enum class ResetMode { clear_discovery, preserve_discovery };
struct CommandBatch {
  std::array<double, kAxes> positions{};
  std::array<double, kAxes> velocities{};
  std::array<double, kAxes> accelerations{};
  std::array<double, kAxes> lookahead_positions{};
  double remaining_time_sec{};
  std::uint64_t control_generation{};
  bool velocities_valid{false};
  Time created{}, expires{};
  std::uint64_t sequence{};
  std::uint64_t configuration_generation{};
  Source source{Source::none};
};
struct TcpStreamingSnapshot {
  std::array<double,6> reference{}, target{}, shaft_rpm{};
  Time reference_created{};
  std::uint64_t generation{};
  bool command_available{};
};
struct BusConfig {
  std::string interface{"can0"};
  unsigned bitrate{500000};
  std::chrono::milliseconds cycle{kStreamingCycle}, response_timeout{20}, feedback_timeout{60}, command_timeout{60};
  bool use_ack{false};
  unsigned rx_poll_timeout_ms{1};
  double utilization_limit{0.5};
  bool motion_enabled{false};
  bool commissioning_verified{false};
  bool synchronization_verified{false};
  bool stop_verified{false};
  bool source_lease_required{false};
  std::chrono::milliseconds source_lease_timeout{200};
  std::array<AxisConfig, kAxes> axes{};
  unsigned manual_max_rpm{30};
  bool program_enabled{}, program_stop_verified{}, program_sync_verified{}, program_staged_reset_verified{};
  bool tcp_jog_enabled{};
  std::string program_revision;  // Stare API: pole ignorowane; obowiązuje zatwierdzony RobotProfile.
  std::chrono::milliseconds program_ack_timeout{kProgramAckTimeout}, program_position_period{kProgramPositionPeriod};
  std::array<double,kAxes> program_position_tolerance{
      kProgramPositionTolerance,kProgramPositionTolerance,kProgramPositionTolerance,
      kProgramPositionTolerance,kProgramPositionTolerance,kProgramPositionTolerance};
  std::array<StreamingParameters,kAxes> program_streaming{};

};
struct BusStatistics {
  std::uint64_t tx{}, rx{}, invalid{}, late{}, timeouts{}, errors{}, deferred{}, missed_cycles{}, committed{}, dropped_commands{};
  Aggregate rtt, rx_interval_ms, cycle_ms, feedback_span_ms, feedback_round_ms;
  Aggregate ack_margin_ms, sync_lateness_ms;
  double scheduled_utilization{}, peak_scheduled_utilization{}, cycle_bits{}, stop_elapsed_ms{};
  std::uint64_t completion_reports{}, endstop_reports{};
  bool stop_pending{};
  unsigned stop_f7_acks{}, stop_zero_acks{}, stop_rejected{}, stop_failed{};
};
struct BusSnapshot {
  std::array<DeviceState, 2048> devices{};
  BusStatistics statistics;
  bool open{}, scanning{}, active{}, fault{};
  unsigned checked{};
  Source source{Source::none};
  std::string message;
};
struct ControlState {
  std::array<Sample<double>, kAxes> positions{};
  std::array<JointConversion,kAxes> conversions{};
  std::uint64_t configuration_generation{};
  std::uint64_t control_generation{};
  bool configuration_ready{};
  bool active{}, fault{}, program_owned{};
  Source source{Source::none};
};

// Wszystkie metody silnika wywołuje jeden wątek właściciela. Czas jest wstrzykiwany
// do testów deterministycznych, bez ROS, USB ani sleep().
class CanBusManager {
 public:
  using TimeSource = std::function<Time()>;
  CanBusManager(BusConfig config, std::unique_ptr<Transport> transport, TimeSource clock = {});
  ~CanBusManager();
  ManualMotorController &manual() { return *manual_; }
  void open(Time now);
  void close();
  void tick(Time now);
  bool start_scan();
  bool service_allowed() const;
  bool reset_allowed() const;
  bool activate(Source source, Time now);
  void configure_tcp_jog(bool enabled) { config_.tcp_jog_enabled=enabled; }
  std::uint64_t begin_tcp_jog(const std::string &session,const std::string &revision,
      const std::string &model,std::uint64_t sequence,Time issued,Time now);
  bool tcp_jog_control(const std::string &session,std::uint64_t generation,std::uint64_t sequence,
      Time issued,Time now,bool brake);
  bool finish_tcp_jog(const std::string &session,std::uint64_t generation);
  bool brake_tcp_jog(const std::string &session,std::uint64_t generation);
  bool tcp_jog_available(Time now) const;
  TcpStreamingSnapshot tcp_streaming_snapshot() const;
  bool submit(const CommandBatch &batch);
  void renew_source_lease(Source source, Time now);
  void stop(const std::string &reason, bool critical = true);
  void clear_fault();
  void reset_session(ResetMode mode = ResetMode::clear_discovery);
  BusSnapshot snapshot() const;
  ControlState control_state() const;
  // Czytelnik RT przekazuje adres ze swojego snapshotu; nie czyta zmiennego config_.
  AxisFeedback encoder_feedback(unsigned address) const { return transport_->position_feedback(address); }
  const BusConfig &config() const { return config_; }
  static double position_to_joint(const AxisConfig &axis, std::int64_t ticks);
  static std::int32_t joint_to_position(const AxisConfig &axis, double position);
  void configure_program_authorization(const BusConfig &authorization);
  void update_motion_model(const std::string &model);
  void update_motion_settings(const std::string &revision, bool allow_missing, Time now);
  MotionProgramStatus prepare_program(PreparedMotionProgram program, Time now);
  bool execute_program(const std::string &id, const std::string &key, Time now);
  MotionProgramStatus program_status() const { return executor_.status; }
  std::string recovery_state_name() const;
  const std::string &recovery_message() const { return recovery_.message; }
  unsigned recovery_pending_mask() const { return recovery_.pending; }
  std::size_t drain_shaft_trace(ShaftPositionSample *out,std::size_t capacity) { return shaft_trace_.drain(out,capacity); }
  std::uint64_t shaft_trace_dropped() const { return shaft_trace_.dropped(); }
  void require_configuration();
  ConfigurationResult configure(ConfigurationPhase phase, const std::string &operation,
      const std::string &expected, const RobotProfile &profile, Time now);

 private:
  friend class ManualMotorController;
  void shaft_reference_changed(unsigned axis);
  void record_shaft_position(unsigned axis,std::int64_t ticks,Time stamp,std::int64_t kernel_utc_ns);
  ShaftTraceQueue shaft_trace_;
  std::array<std::uint64_t,6> shaft_sequences_{}, shaft_segments_{};
  std::array<std::uint32_t,6> shaft_flags_{};
  enum class Kind { identity, mode, response_mode, sync_mode, microstep, position, enable, stall, motor, speed, watchdog, manual, program_setup, recovery };
  struct Pending {
    bool used{false};
    std::uint16_t address{};
    Kind kind{};
    std::uint8_t command{};
    std::size_t length{};
    Time sent{}, deadline{};
  };
  bool send_query(std::uint16_t address, Kind kind, Time now, bool critical);
  bool send_manual(std::uint16_t address, const std::vector<std::uint8_t> &data,
                   std::size_t response_length, Time now, bool urgent,
                   std::chrono::milliseconds timeout = std::chrono::milliseconds{0});
  bool can_send(std::uint16_t address, Time now) const;
  bool reserve(std::size_t request_length, std::size_t response_length, bool critical, unsigned extra_responses = 0);
  void receive(Time now);
  void expire(Time now);
  bool decode(Pending &pending, const Frame &frame, Time stamp);
  void next_scan(std::uint16_t address);
  void fault(const std::string &message, bool critical = true);
  void disable_enable_after_fault();
  void emergency_stop();
  void send_zero_stop(Time now);
  void begin_recovery(const std::string &reason,bool force_disable,Time now);
  void send_recovery_stop_phase(unsigned phase,Time now);
  void schedule_recovery(Time now);
  void recovery_reply(const Frame &frame,Time stamp);
  bool send_recovery(std::uint16_t address,const std::vector<std::uint8_t> &data,
      std::size_t response_length,Time now);
  void recovery_failed(const std::string &reason);
  bool recovery_standstill(Time now,bool capture);
  void schedule_fault_diagnostics(Time now);
  void reconnect(Time now);
  void transport_lost(Time now,const std::string &reason);
  Time current_time(Time fallback) const;
  bool phased_program() const;
  unsigned feedback_group() const;
  void rebuild_axis_lookup();
  double mandatory_cycle_bits() const;
  double stream_cycle_bits(unsigned participants) const;
  void send_stream_payload(unsigned joint,const std::vector<std::uint8_t> &payload);
  Sample<double> streaming_feedback_position(unsigned joint,Time now) const;
  bool feedback_good(Time now) const;
  std::string feedback_failure_reason(Time now) const;
  void schedule_feedback(Time now);
  void schedule_background(Time now);
  void schedule_motion(Time now);
  void schedule_program(Time now);
  void schedule_synchronized_program(Time now);
  bool synchronized_reply(const Frame &frame,Time stamp);
  void synchronized_confirmation(const Frame &frame,Time now);
  bool synchronized_query_window(Time now) const;
  bool sync_recovery_required_{};
  bool unverified_sync_buffer_{};
  std::array<StreamingState,kAxes> staged_streaming_states_{};
  bool program_feedback_good(Time now) const;
  void begin_program_check(Time now);
  bool check_program_positions(Time now);
  void program_setup_reply(const Frame &frame, Time now);

  void start_cycle(Time now);
  std::size_t axis_index(std::uint16_t address) const;
  MotionExecutor executor_;
  std::string motion_settings_revision_, motion_model_;
  bool allow_missing_motion_{}, program_owned_{};
  Time motion_settings_stamp_{};
  std::uint64_t program_generation_{};
  std::array<std::string,64> execution_keys_{};
  std::size_t execution_key_cursor_{};
  BusConfig config_;
  TimeSource clock_;
  std::unique_ptr<Transport> transport_;
  std::unique_ptr<ManualMotorController> manual_;
  std::array<DeviceState, 2048> devices_{};
  std::array<std::uint8_t, 2048> axis_lookup_{};
  std::array<std::uint16_t, kScanMaximumAddress> known_addresses_{};
  std::size_t known_count_{};
  std::vector<std::uint8_t> query_payload_{0,0,0};
  const std::vector<std::uint8_t> sync_payload_{0x4b,0x4b};
  std::array<std::vector<std::uint8_t>,kAxes> stream_payloads_{};
  std::array<StreamingState,kAxes> streaming_states_{};
  // Wyłącznie odczyty i konfiguracja. Ruch nigdy nie zajmuje slotu ACK.
  std::array<Pending, 4> pending_{};
  std::array<Time, 2048> quarantine_{};
  std::array<Time, 2048> next_position_{}, next_enable_{}, next_stall_{}, next_motor_{}, next_mode_{}, next_speed_{};
  std::array<double, kAxes> committed_positions_{}, previous_velocity_{};
  BusStatistics stats_;
  std::optional<CommandBatch> latest_;
  std::uint64_t last_sequence_{};
  Time cycle_start_{}, next_cycle_{}, last_rx_{}, last_command_{}, lease_stamp_{}, last_tick_{}, last_cycle_wake_{};
  std::uint64_t cycle_number_{};
  std::uint64_t last_fault_diagnostic_cycle_{std::numeric_limits<std::uint64_t>::max()};
  std::size_t diagnostic_transactions_{};
  std::size_t fault_diagnostic_cursor_{};
  unsigned scan_address_{1}, checked_{}, cursor_{1};
  bool scanning_{false}, active_{false}, fault_{false}, stop_sent_{false}, cycle_initialized_{false}, motion_this_cycle_{false}, feedback_recorded_{false};
  enum class RecoveryPhase { idle, stopping, verifying_disabled, overwriting, restoring, verifying, idle_disabled, failed };
  struct Recovery {
    RecoveryPhase phase{RecoveryPhase::idle};
    std::string message;
    unsigned required{}, pending{};
    unsigned axis{}, step{}, stop_phase{};
    std::array<std::uint16_t,kAxes> addresses{};
    bool waiting{}, force_disable{}, staged_buffer_seen{};
    Time started{}, deadline{}, verify_after{}, stop_next{};
    std::array<Sample<std::int64_t>,kAxes> anchors{};
    std::array<Time,kAxes> watchdog_pause_until{};
  } recovery_;
  std::array<Time,kAxes> fault_probe_after_{};
  Time next_reconnect_{};
  unsigned reconnect_attempt_{};
  Source source_{Source::none};
  std::string tcp_owner_;
  std::uint64_t control_generation_{}, tcp_sequence_{};
  Time tcp_lease_{}, tcp_started_{}, tcp_last_stop_{};
  bool tcp_braking_{};
  double spent_bits_{}, background_bits_{};
  struct StopSequence {
    enum class Phase { idle, f7, zero, done } phase{Phase::idle};
    std::array<std::uint16_t, kAxes> addresses{};
    unsigned required{}, replied{};
    Time begun{}, sent{}, deadline{}, final_deadline{};
  } stop_sequence_;
  std::string message_{"Oczekiwanie na magistralę CAN"};
  RobotProfile profile_{}, candidate_profile_{}, previous_profile_{};
  std::array<AxisConfig, kAxes> previous_axes_{};
  bool configuration_ready_{true}, configuration_locked_{false};
  bool configuration_bootstrap_{false}, scan_after_bootstrap_{true};
  std::array<bool, kAxes> calibration_invalid_{}, previous_calibration_invalid_{};
  std::array<bool,kAxes> configuration_anchored_{};
  std::array<std::int64_t,kAxes> configuration_ticks_{};
  std::array<std::uint64_t,kAxes> configuration_segments_{};
  std::uint64_t configuration_generation_{};
  std::string configuration_operation_, configuration_stage_{"finished"};
  std::array<std::string, 64> retired_configuration_operations_{};
  std::size_t retired_configuration_cursor_{};
};
}  // namespace mks_servo_can
