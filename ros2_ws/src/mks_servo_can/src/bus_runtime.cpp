#include "mks_servo_can/bus_runtime.hpp"
#include <cstdio>
#include "mks_servo_can/trace.hpp"
#include <stdexcept>

namespace mks_servo_can {
namespace {
std::unique_ptr<Transport> with_priority(std::unique_ptr<Transport> transport,int priority) {
  transport->set_realtime_priority(priority);return transport;
}
}
BusRuntime::BusRuntime(BusConfig config, std::unique_ptr<Transport> transport)
    : realtime_priority_(config.io_realtime_priority),
      manager_(config, with_priority(std::move(transport),config.io_realtime_priority), Clock::now) {}
std::string BusRuntime::realtime_status() const {
  std::string transport;
  {std::lock_guard<PriorityInheritanceMutex> guard(mutex_);transport=manager_.transport_realtime_status();}
  if(io_realtime_status_.empty()) return transport.empty() ? "" : "RX: "+transport;
  return "IO: "+io_realtime_status_+(transport.empty() ? "" : "; RX: "+transport);
}
BusRuntime::~BusRuntime() { shutdown(); }
void BusRuntime::start(bool scan_on_start) {
  if (running_) throw std::logic_error("Manager CAN już działa");
  { std::lock_guard<PriorityInheritanceMutex> guard(mutex_); manager_.open(Clock::now()); if (scan_on_start) manager_.start_scan();feedback_.publish(manager_.control_state()); }
  running_ = true;
  worker_ = std::thread([this] {
    reduce_timer_slack();
    Time previous{},planned{};auto deadline=Clock::now();
    while (running_) {
      const auto wake = Clock::now();Time exact{};
      { std::lock_guard<PriorityInheritanceMutex> guard(mutex_);
        const auto begin = Clock::now();CommandBatch command;
        if(commands_.take(command,seen_command_)) manager_.submit(command);
        manager_.tick(begin);feedback_.publish(manager_.control_state());exact=manager_.next_deadline();const auto end = Clock::now();
        if (previous != Time{} && begin - previous > std::chrono::milliseconds(5)) {
          TraceEvent e; e.event = "io_loop_gap"; e.duration_ns = trace_ns(begin) - trace_ns(previous);
          e.request_ns = trace_ns(wake); e.deadline_ns = trace_ns(begin);
          // Rozróżnienie przyczyny: spóźnione wybudzenie z uśpienia (planowany termin co 1 ms) albo czekanie na mutex.
          const double late_ms=planned==Time{} ? -1.0 : std::chrono::duration<double,std::milli>(wake-planned).count();
          const double lock_ms=std::chrono::duration<double,std::milli>(begin-wake).count();
          std::snprintf(e.detail.data(),e.detail.size(),"wybudzenie spóźnione o %.3f ms względem planu; mutex %.3f ms",late_ms,lock_ms);
          trace(e);
        }
        if (end - begin > std::chrono::milliseconds(2)) {
          TraceEvent e; e.event = "io_tick_slow"; e.duration_ns = trace_ns(end) - trace_ns(begin); trace(e);
        }
        previous = begin;
      }
      deadline+=std::chrono::milliseconds(1);
      // Termin SYNC poza siatką 1 ms: wybudzenie dokładnie na Tk zamiast do 1 ms później.
      if(exact!=Time{} && exact<deadline) deadline=exact;
      const auto current=Clock::now();
      if(deadline<current) deadline=current;
      planned=deadline;
      std::this_thread::sleep_until(deadline);
    }
  });
  io_realtime_status_=set_realtime_priority(worker_,realtime_priority_);
  if(!io_realtime_status_.empty()) trace_note("realtime_io",io_realtime_status_);
}
void BusRuntime::shutdown() {
  running_ = false;
  if (worker_.joinable()) worker_.join();
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_); manager_.close();
}
bool BusRuntime::scan() { std::lock_guard<PriorityInheritanceMutex> guard(mutex_); return manager_.start_scan(); }
void BusRuntime::reset_session(ResetMode mode) { std::lock_guard<PriorityInheritanceMutex> guard(mutex_); manager_.reset_session(mode); }
bool BusRuntime::reset_allowed() { std::lock_guard<PriorityInheritanceMutex> guard(mutex_); return manager_.reset_allowed(); }
bool BusRuntime::activate(Source source) { std::lock_guard<PriorityInheritanceMutex> guard(mutex_);const auto accepted=manager_.activate(source,Clock::now());feedback_.publish(manager_.control_state());return accepted; }
void BusRuntime::configure_tcp_jog(bool enabled) { std::lock_guard<PriorityInheritanceMutex> guard(mutex_);manager_.configure_tcp_jog(enabled); }
TcpStreamingSnapshot BusRuntime::tcp_streaming_snapshot() {std::lock_guard<PriorityInheritanceMutex> guard(mutex_);return manager_.tcp_streaming_snapshot();}
std::uint64_t BusRuntime::begin_tcp_jog(const std::string &session,const std::string &revision,const std::string &model,std::uint64_t sequence,Time issued,Time *slot_origin) {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_);
  const auto now=Clock::now();
  const auto generation=manager_.begin_tcp_jog(session,revision,model,sequence,issued,now);
  if(slot_origin && generation) *slot_origin=now;
  feedback_.publish(manager_.control_state());return generation;
}
bool BusRuntime::tcp_jog_control(const std::string &session,std::uint64_t generation,std::uint64_t sequence,Time issued) {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_);return manager_.tcp_jog_control(session,generation,sequence,issued,Clock::now());
}
bool BusRuntime::append_tcp_points(const std::string &session,std::uint64_t generation,const std::vector<ProgramPoint> &points,bool closed) {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_);return manager_.append_tcp_points(session,generation,points,closed,Clock::now());
}
bool BusRuntime::cut_tcp_jog(const std::string &session,std::uint64_t generation,const std::string &reason) {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_);return manager_.cut_tcp_jog(session,generation,reason,Clock::now());
}
TcpTrajectoryState BusRuntime::tcp_trajectory() {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_);return manager_.tcp_trajectory(Clock::now());
}
bool BusRuntime::finish_tcp_jog(const std::string &session,std::uint64_t generation) {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_);return manager_.finish_tcp_jog(session,generation);
}
bool BusRuntime::request_drive_parameters() {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_);return manager_.request_drive_parameters();
}
std::uint64_t BusRuntime::submit_parameter_job(std::uint16_t address,std::uint8_t code,int sub,bool write,
    const std::vector<std::uint8_t> &value,std::string &reason) {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_);
  return manager_.submit_parameter_job(address,code,sub,write,value,Clock::now(),reason);
}
CanBusManager::ParameterJobResult BusRuntime::parameter_job(std::uint64_t id) {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_);return manager_.parameter_job(id);
}
bool BusRuntime::request_microstep(unsigned axis,std::uint16_t microstep) {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_);return manager_.request_microstep(axis,microstep,Clock::now());
}
bool BusRuntime::tcp_jog_available() {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_);return manager_.tcp_jog_available(Clock::now());
}
void BusRuntime::stop(const std::string &reason, bool critical) { std::lock_guard<PriorityInheritanceMutex> guard(mutex_); manager_.stop(reason, critical); }
void BusRuntime::renew_source_lease(Source source, Time created) {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_);
  const auto now = Clock::now();
  if (created > now || now - created >= manager_.config().source_lease_timeout) return;
  manager_.renew_source_lease(source, created);
}
bool BusRuntime::try_submit(const CommandBatch &batch) {
  if(!running_.load(std::memory_order_acquire)) return false;
  // Skrzynka jest single-writer. Nawet spóźniony callback poprzedniego źródła
  // nie może przeplatać zapisu z nowym właścicielem ani nadpisywać jego referencji.
  if(publishing_.test_and_set(std::memory_order_acquire)) return false;
  ControlState state;
  const bool accepted=feedback_.read(state) && state.active && !state.fault && state.configuration_ready &&
      state.source==batch.source && state.configuration_generation==batch.configuration_generation &&
      batch.control_generation==state.control_generation;
  if(accepted) commands_.publish(batch);
  publishing_.clear(std::memory_order_release);return accepted;
}
bool BusRuntime::try_read(ControlState &state) {
  return feedback_.read(state);
}
BusSnapshot BusRuntime::snapshot() { std::lock_guard<PriorityInheritanceMutex> guard(mutex_); return manager_.snapshot(); }
ManualState BusRuntime::manual_snapshot() { std::lock_guard<PriorityInheritanceMutex> guard(mutex_); return manager_.manual().snapshot(Clock::now()); }
bool BusRuntime::manual_enable(unsigned axis, bool enabled) {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_); return manager_.manual().set_enable(axis, enabled, Clock::now());
}
bool BusRuntime::manual_zero(unsigned axis) {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_); return manager_.manual().zero(axis, Clock::now());
}
bool BusRuntime::manual_enable_all(bool enabled) {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_);return manager_.manual().set_enable_all(enabled,Clock::now());
}
bool BusRuntime::manual_jog(JogAction action, unsigned axis, int direction, unsigned rpm,
    const std::string &session, std::uint64_t sequence, Time issued) {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_);
  const auto accepted = manager_.manual().jog(action, axis, direction, rpm, session, sequence, issued, Clock::now());
  TraceEvent e; e.event = "manual_command"; e.axis = axis; e.command = static_cast<unsigned>(action);
  e.request_ns = trace_ns(issued); e.flags = accepted; e.duration_ns = trace_ns(Clock::now()) - trace_ns(issued); trace(e);
  return accepted;
}
bool BusRuntime::joint_jog(JogAction action, unsigned axis, int direction, double velocity,
    const std::string &revision, const std::string &session, std::uint64_t sequence, Time issued) {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_);
  return manager_.manual().jog_joint(action, axis, direction, velocity, revision, session, sequence, issued, Clock::now());
}
void BusRuntime::configure_program_authorization(const BusConfig &a) { std::lock_guard<PriorityInheritanceMutex> guard(mutex_);manager_.configure_program_authorization(a); }
bool BusRuntime::set_execution_tuning(const ExecutionTuning &tuning, std::string &reason) { std::lock_guard<PriorityInheritanceMutex> guard(mutex_);return manager_.set_execution_tuning(tuning,reason); }
void BusRuntime::update_motion_model(const std::string &model) { std::lock_guard<PriorityInheritanceMutex> guard(mutex_);manager_.update_motion_model(model); }
void BusRuntime::update_motion_settings(const std::string &revision,bool allow_missing) { std::lock_guard<PriorityInheritanceMutex> guard(mutex_);manager_.update_motion_settings(revision,allow_missing,Clock::now()); }
MotionProgramStatus BusRuntime::prepare_program(PreparedMotionProgram program) { std::lock_guard<PriorityInheritanceMutex> guard(mutex_);return manager_.prepare_program(std::move(program),Clock::now()); }
bool BusRuntime::execute_program(const std::string &id,const std::string &key) { std::lock_guard<PriorityInheritanceMutex> guard(mutex_);return manager_.execute_program(id,key,Clock::now()); }
MotionProgramStatus BusRuntime::program_status() { std::lock_guard<PriorityInheritanceMutex> guard(mutex_);return manager_.program_status(); }
void BusRuntime::require_configuration() {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_); manager_.require_configuration();
}
ConfigurationResult BusRuntime::configure(ConfigurationPhase phase, const std::string &operation,
    const std::string &expected, const RobotProfile &profile) {
  std::lock_guard<PriorityInheritanceMutex> guard(mutex_);
  const auto result=manager_.configure(phase, operation, expected, profile, Clock::now());
  feedback_.publish(manager_.control_state());return result;
}
}  // namespace mks_servo_can
