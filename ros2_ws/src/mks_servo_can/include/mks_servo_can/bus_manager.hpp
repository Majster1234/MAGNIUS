#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include "mks_servo_can/protocol.hpp"
#include "mks_servo_can/socketcan.hpp"

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
  Time last_ok{};
  Time position_sent{};
  Sample<std::int64_t> position;
  Sample<bool> enable, stall;
  Sample<std::uint8_t> mode, motor_state;
  Sample<std::int16_t> speed;
  Aggregate rtt;
  double last_rtt_ms{};
  std::uint64_t timeouts{}, invalid{};
  unsigned consecutive_timeouts{};
};
struct AxisConfig {
  std::uint16_t address{};
  double gear_ratio{}, direction{}, offset{}, lower{}, upper{}, max_velocity{}, max_acceleration{}, tracking_error{};
  std::uint8_t acceleration_code{};
};
enum class Source { none, trajectory, jog };
struct CommandBatch {
  std::array<double, kAxes> positions{};
  Time created{}, expires{};
  std::uint64_t sequence{};
  Source source{Source::none};
};
struct BusConfig {
  std::string interface{"can0"};
  unsigned bitrate{500000};
  std::chrono::milliseconds cycle{20}, response_timeout{20}, feedback_timeout{60}, command_timeout{60};
  double utilization_limit{0.5};
  bool motion_enabled{false};
  bool commissioning_verified{false};
  bool synchronization_verified{false};
  bool stop_verified{false};
  bool source_lease_required{false};
  std::chrono::milliseconds source_lease_timeout{200};
  std::array<AxisConfig, kAxes> axes{};
  unsigned manual_max_rpm{30};
};
struct BusStatistics {
  std::uint64_t tx{}, rx{}, invalid{}, late{}, timeouts{}, errors{}, deferred{}, missed_cycles{}, committed{}, dropped_commands{};
  Aggregate rtt, rx_interval_ms, cycle_ms, feedback_span_ms, feedback_round_ms;
  double scheduled_utilization{};
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
  bool active{}, fault{};
  Source source{Source::none};
};

// Wszystkie metody silnika wywołuje jeden wątek właściciela. Czas jest wstrzykiwany
// do testów deterministycznych, bez ROS, USB ani sleep().
class CanBusManager {
 public:
  CanBusManager(BusConfig config, std::unique_ptr<Transport> transport);
  ~CanBusManager();
  ManualMotorController &manual() { return *manual_; }
  void open(Time now);
  void close();
  void tick(Time now);
  bool start_scan();
  bool service_allowed() const;
  bool activate(Source source, Time now);
  bool submit(const CommandBatch &batch);
  void renew_source_lease(Source source, Time now);
  void stop(const std::string &reason);
  void clear_fault();
  void reset_session();
  BusSnapshot snapshot() const;
  ControlState control_state() const;
  const BusConfig &config() const { return config_; }
  static double position_to_joint(const AxisConfig &axis, std::int64_t ticks);
  static std::int32_t joint_to_position(const AxisConfig &axis, double position);

 private:
  friend class ManualMotorController;
  enum class Kind { identity, mode, position, enable, stall, motor, speed, stage, manual };
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
  bool reserve(std::size_t request_length, std::size_t response_length, bool critical);
  void receive(Time now);
  void expire(Time now);
  bool decode(Pending &pending, const Frame &frame, Time stamp);
  void next_scan(std::uint16_t address);
  void fault(const std::string &message);
  void emergency_stop();
  bool feedback_good(Time now) const;
  void schedule_feedback(Time now);
  void schedule_background(Time now);
  void schedule_motion(Time now);
  void start_cycle(Time now);
  std::size_t axis_index(std::uint16_t address) const;
  BusConfig config_;
  std::unique_ptr<Transport> transport_;
  std::unique_ptr<ManualMotorController> manual_;
  std::array<DeviceState, 2048> devices_{};
  std::array<Pending, 4> pending_{};
  std::array<Time, 2048> quarantine_{};
  std::array<Time, 2048> next_position_{}, next_enable_{}, next_stall_{}, next_motor_{}, next_mode_{}, next_speed_{};
  std::array<bool, kAxes> staged_{}, sent_stage_{};
  std::array<double, kAxes> committed_positions_{}, previous_velocity_{};
  BusStatistics stats_;
  std::optional<CommandBatch> latest_, inflight_;
  std::uint64_t last_sequence_{};
  Time cycle_start_{}, next_cycle_{}, last_rx_{}, last_command_{}, lease_stamp_{};
  unsigned scan_address_{1}, checked_{}, cursor_{1};
  bool scanning_{false}, active_{false}, fault_{false}, stop_sent_{false}, cycle_initialized_{false}, motion_this_cycle_{false}, feedback_recorded_{false};
  Source source_{Source::none};
  double spent_bits_{};
  std::string message_{"Oczekiwanie na magistralę CAN"};
};
}  // namespace mks_servo_can
