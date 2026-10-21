#include "mks_servo_can/bus_runtime.hpp"
#include "mks_servo_can/trace.hpp"
#include <stdexcept>

namespace mks_servo_can {
BusRuntime::BusRuntime(BusConfig config, std::unique_ptr<Transport> transport)
    : manager_(std::move(config), std::move(transport)) {}
BusRuntime::~BusRuntime() { shutdown(); }
void BusRuntime::start(bool scan_on_start) {
  if (running_) throw std::logic_error("Manager CAN już działa");
  { std::lock_guard<std::mutex> guard(mutex_); manager_.open(Clock::now()); if (scan_on_start) manager_.start_scan(); }
  running_ = true;
  worker_ = std::thread([this] {
    Time previous{};auto deadline=Clock::now();
    while (running_) {
      const auto wake = Clock::now();
      { std::lock_guard<std::mutex> guard(mutex_);
        const auto begin = Clock::now(); manager_.tick(begin); const auto end = Clock::now();
        if (previous != Time{} && begin - previous > std::chrono::milliseconds(5)) {
          TraceEvent e; e.event = "io_loop_gap"; e.duration_ns = trace_ns(begin) - trace_ns(previous);
          e.request_ns = trace_ns(wake); e.deadline_ns = trace_ns(begin); trace(e);
        }
        if (end - begin > std::chrono::milliseconds(2)) {
          TraceEvent e; e.event = "io_tick_slow"; e.duration_ns = trace_ns(end) - trace_ns(begin); trace(e);
        }
        previous = begin;
      }
      deadline+=std::chrono::milliseconds(1);
      const auto current=Clock::now();
      if(deadline<current) deadline=current;
      std::this_thread::sleep_until(deadline);
    }
  });
}
void BusRuntime::shutdown() {
  running_ = false;
  if (worker_.joinable()) worker_.join();
  std::lock_guard<std::mutex> guard(mutex_); manager_.close();
}
bool BusRuntime::scan() { std::lock_guard<std::mutex> guard(mutex_); return manager_.start_scan(); }
void BusRuntime::reset_session() { std::lock_guard<std::mutex> guard(mutex_); manager_.reset_session(); }
bool BusRuntime::reset_allowed() { std::lock_guard<std::mutex> guard(mutex_); return manager_.service_allowed(); }
bool BusRuntime::activate(Source source) { std::lock_guard<std::mutex> guard(mutex_); return manager_.activate(source, Clock::now()); }
void BusRuntime::stop(const std::string &reason) { std::lock_guard<std::mutex> guard(mutex_); manager_.stop(reason); }
void BusRuntime::renew_source_lease(Source source, Time created) {
  std::lock_guard<std::mutex> guard(mutex_);
  const auto now = Clock::now();
  if (created > now || now - created >= manager_.config().source_lease_timeout) return;
  manager_.renew_source_lease(source, created);
}
bool BusRuntime::try_submit(const CommandBatch &batch) {
  std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
  return guard.owns_lock() && manager_.submit(batch);
}
bool BusRuntime::try_read(ControlState &state) {
  std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
  if (!guard.owns_lock()) return false;
  state = manager_.control_state(); return true;
}
BusSnapshot BusRuntime::snapshot() { std::lock_guard<std::mutex> guard(mutex_); return manager_.snapshot(); }
ManualState BusRuntime::manual_snapshot() { std::lock_guard<std::mutex> guard(mutex_); return manager_.manual().snapshot(Clock::now()); }
bool BusRuntime::manual_enable(unsigned axis, bool enabled) {
  std::lock_guard<std::mutex> guard(mutex_); return manager_.manual().set_enable(axis, enabled, Clock::now());
}
bool BusRuntime::manual_zero(unsigned axis) {
  std::lock_guard<std::mutex> guard(mutex_); return manager_.manual().zero(axis, Clock::now());
}
bool BusRuntime::manual_jog(JogAction action, unsigned axis, int direction, unsigned rpm,
    const std::string &session, std::uint64_t sequence, Time issued) {
  std::lock_guard<std::mutex> guard(mutex_);
  const auto accepted = manager_.manual().jog(action, axis, direction, rpm, session, sequence, issued, Clock::now());
  TraceEvent e; e.event = "manual_command"; e.axis = axis; e.command = static_cast<unsigned>(action);
  e.request_ns = trace_ns(issued); e.flags = accepted; e.duration_ns = trace_ns(Clock::now()) - trace_ns(issued); trace(e);
  return accepted;
}
bool BusRuntime::joint_jog(JogAction action, unsigned axis, int direction, double velocity,
    const std::string &revision, const std::string &session, std::uint64_t sequence, Time issued) {
  std::lock_guard<std::mutex> guard(mutex_);
  return manager_.manual().jog_joint(action, axis, direction, velocity, revision, session, sequence, issued, Clock::now());
}
void BusRuntime::configure_program_authorization(const BusConfig &a) { std::lock_guard<std::mutex> guard(mutex_);manager_.configure_program_authorization(a); }
void BusRuntime::update_motion_model(const std::string &model) { std::lock_guard<std::mutex> guard(mutex_);manager_.update_motion_model(model); }
void BusRuntime::update_motion_settings(const std::string &revision,bool allow_missing) { std::lock_guard<std::mutex> guard(mutex_);manager_.update_motion_settings(revision,allow_missing,Clock::now()); }
MotionProgramStatus BusRuntime::prepare_program(PreparedMotionProgram program) { std::lock_guard<std::mutex> guard(mutex_);return manager_.prepare_program(std::move(program),Clock::now()); }
bool BusRuntime::execute_program(const std::string &id,const std::string &key) { std::lock_guard<std::mutex> guard(mutex_);return manager_.execute_program(id,key,Clock::now()); }
MotionProgramStatus BusRuntime::program_status() { std::lock_guard<std::mutex> guard(mutex_);return manager_.program_status(); }
void BusRuntime::require_configuration() {
  std::lock_guard<std::mutex> guard(mutex_); manager_.require_configuration();
}
ConfigurationResult BusRuntime::configure(ConfigurationPhase phase, const std::string &operation,
    const std::string &expected, const RobotProfile &profile) {
  std::lock_guard<std::mutex> guard(mutex_); return manager_.configure(phase, operation, expected, profile, Clock::now());
}
}  // namespace mks_servo_can
