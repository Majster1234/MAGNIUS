#pragma once
#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/idle_enable_monitor.hpp"

namespace mks_servo_can {
struct ManualAxisState {
  unsigned axis{};
  bool available{}, prepared{}, can_jog{}, can_zero{};
  Sample<bool> enable;
  Sample<double> position, speed;
  Sample<double> joint_position;
  bool configured{}, calibration_valid{};
  double minimum_velocity{}, maximum_velocity{}, joint_lower{}, joint_upper{};
  std::string configuration_message;
  std::string motion{"stan nieznany"};
};
struct ManualState {
  std::array<ManualAxisState, kAxes> axes;
  bool busy{}, fault{};
  unsigned axis{}, max_rpm{30};
  std::string operation, message{"Sterowanie ręczne gotowe"};
  std::string configuration_revision{"initial"};
  std::string recovery_state{"idle"}, recovery_message;
  std::uint8_t recovery_pending_mask{};
};
enum class JogAction { start, keepalive, stop };

// Sekwencje napędu i stan ruchu. Nie posiada socketu ani własnego wątku.
class ManualMotorController {
 public:
  explicit ManualMotorController(CanBusManager &bus) : bus_(bus) {}
  bool set_enable(unsigned axis, bool enabled, Time now);
  bool set_enable_all(bool enabled, Time now);
  bool group_reply(unsigned axis,const std::vector<std::uint8_t> &data,Time stamp);
  bool zero(unsigned axis, Time now);
  bool jog(JogAction action, unsigned axis, int direction, unsigned rpm,
           const std::string &session, std::uint64_t sequence, Time issued, Time now);
  bool jog_joint(JogAction action, unsigned axis, int direction, double velocity,
      const std::string &revision, const std::string &session, std::uint64_t sequence, Time issued, Time now);
  void configuration_changed(const std::array<bool, kAxes> &changed);
  bool configuration_stopped(unsigned axis, Time now) const;
  bool dispatch_stopped(unsigned axis, Time now) const;
  bool calibration_valid(unsigned axis) const { return calibrated_[axis - 1]; }
  void tick(Time now);
  void reply(const std::vector<std::uint8_t> &data, Time stamp);
  void timeout();
  void transport_fault();
  void recovery_complete();
  void abort(const std::string &reason, bool critical = true) { fail(reason, critical); }
  std::uint16_t microstep(unsigned axis) const { return microstep_.at(axis-1); }
  bool prepared(unsigned axis) const { return prepared_.at(axis-1); }
  void invalidate_program_handover() { prepared_.fill(false); profile_verified_.fill(false); }
  bool busy() const { return phase_ != Phase::idle || group_active_; }
  unsigned axis() const { return axis_; }
  bool fast_feedback(unsigned axis) const { return axis <= kAxes && axis && prepared_[axis - 1]; }
  ManualState snapshot(Time now) const;
 private:
  enum class Phase { idle, sequence, starting, moving, stopping, fault };
  enum class Operation { enable, disable, zero, jog };
  struct Step {
    std::vector<std::uint8_t> data;
    std::size_t length{3};
    int expected{1};
  };
  bool begin(unsigned axis, Operation op, Time now);
  void group_tick(Time now);
  void idle_tick(Time now);
  void add(std::vector<std::uint8_t> data, std::size_t length = 3, int expected = 1);
  void stop(Time now, const std::string &reason);
  void fail(const std::string &message, bool critical = true);
  bool stopped(unsigned axis, Time now, std::chrono::milliseconds window = std::chrono::milliseconds(60)) const;
  bool jog_stopped(unsigned axis, Time now) const;
  bool position_stable(unsigned axis, Time now, std::chrono::milliseconds window = std::chrono::milliseconds(100)) const;
  void observe_positions(Time now);
  bool ready(unsigned axis, Time now) const;
  double brake_distance(unsigned axis, double rpm, Time now) const;
  bool limit_reached(unsigned axis, int direction, double rpm, Time now) const;
  bool jog_impl(JogAction action, unsigned axis, int direction, double rpm,
      const std::string &session, std::uint64_t sequence, Time issued, Time now, bool joint);
  void complete(Time now);
  CanBusManager &bus_;
  std::array<bool, kAxes> prepared_{};
  std::array<bool, kAxes> calibrated_{}, profile_verified_{};
  std::array<std::string, kAxes> configuration_messages_{};
  std::array<std::uint16_t, kAxes> microstep_{};
  std::array<Sample<std::int64_t>, kAxes> stationary_anchor_{};
  std::array<Sample<std::int64_t>, kAxes> jog_rest_anchor_{};
  std::array<Sample<std::int64_t>, kAxes> velocity_previous_{};
  std::array<Sample<double>, kAxes> velocity_{};
  std::array<Step, 24> steps_{};
  std::size_t step_{}, step_count_{};
  unsigned axis_{};
  double rpm_{};
  int direction_{};
  Phase phase_{Phase::idle};
  Operation operation_{Operation::enable};
  bool waiting_{}, stop_sent_{}, disable_after_{};
  Time lease_{}, stop_at_{}, stop_deadline_{}, last_stop_{}, operation_deadline_{};
  Time retry_at_{};
  Time next_trace_{};
  std::string owner_, message_{"Sterowanie ręczne gotowe"};
  std::uint64_t sequence_{};
  bool group_active_{}, group_step_{}, group_enabled_{}, group_committing_{};
  Time group_sent_{}, group_next_{};
  unsigned group_axis_{};
  std::array<bool,kAxes> group_targets_{}, group_done_{};
  IdleEnableMonitor idle_monitor_;
  bool idle_disable_{};
};
}  // namespace mks_servo_can
