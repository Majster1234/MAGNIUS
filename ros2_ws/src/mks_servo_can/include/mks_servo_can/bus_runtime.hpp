#pragma once
#include <atomic>
#include <mutex>
#include <thread>
#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include "mks_servo_can/realtime_mailbox.hpp"

namespace mks_servo_can {
// Właściciel wątku IO. Interfejsy try_* nie czekają na mutex ani transport.
class BusRuntime {
 public:
  explicit BusRuntime(BusConfig config, std::unique_ptr<Transport> transport = std::make_unique<SocketCan>());
  ~BusRuntime();
  void start(bool scan = true);
  void shutdown();
  void reset_session(ResetMode mode = ResetMode::clear_discovery);
  bool scan();
  bool reset_allowed();
  bool activate(Source source);
  void configure_tcp_jog(bool enabled);
  std::uint64_t begin_tcp_jog(const std::string &session,const std::string &revision,
      const std::string &model,std::uint64_t sequence,Time issued);
  bool tcp_jog_control(const std::string &session,std::uint64_t generation,std::uint64_t sequence,Time issued,bool brake);
  bool finish_tcp_jog(const std::string &session,std::uint64_t generation);
  bool brake_tcp_jog(const std::string &session,std::uint64_t generation);
  bool tcp_jog_available();
  void stop(const std::string &reason);
  void renew_source_lease(Source source, Time created);
  bool try_submit(const CommandBatch &batch);
  bool try_read(ControlState &state);
  AxisFeedback encoder_feedback(unsigned address) const { return manager_.encoder_feedback(address); }
  BusConfig config_snapshot() { std::lock_guard<std::mutex> guard(mutex_);return manager_.config(); }
  BusSnapshot snapshot();
  ManualState manual_snapshot();
  std::size_t drain_shaft_trace(ShaftPositionSample *out,std::size_t maximum) { return manager_.drain_shaft_trace(out,maximum); }
  std::uint64_t shaft_trace_dropped() const { return manager_.shaft_trace_dropped(); }
  bool manual_enable(unsigned axis, bool enabled);
  bool manual_enable_all(bool enabled);
  bool manual_zero(unsigned axis);
  bool manual_jog(JogAction action, unsigned axis, int direction, unsigned rpm,
                  const std::string &session, std::uint64_t sequence, Time issued);
  bool joint_jog(JogAction action, unsigned axis, int direction, double velocity,
      const std::string &revision, const std::string &session, std::uint64_t sequence, Time issued);
  void configure_program_authorization(const BusConfig &authorization);
  void update_motion_model(const std::string &model);
  void update_motion_settings(const std::string &revision, bool allow_missing);
  MotionProgramStatus prepare_program(PreparedMotionProgram program);
  bool execute_program(const std::string &id, const std::string &key);
  MotionProgramStatus program_status();
  void require_configuration();
  ConfigurationResult configure(ConfigurationPhase phase, const std::string &operation,
      const std::string &expected, const RobotProfile &profile);
 private:
  std::mutex mutex_;
  CanBusManager manager_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  CommandMailbox commands_;
  std::atomic_flag publishing_=ATOMIC_FLAG_INIT;
  ControlMailbox feedback_;
  std::uint64_t seen_command_{};
};
}  // namespace mks_servo_can
