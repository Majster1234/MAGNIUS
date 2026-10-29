#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <optional>
#include <chrono>

namespace mks_servo_can {
struct AxisConfig;
struct ProgramPoint {
  double seconds{};
  std::array<double,6> positions{}, velocities{}, accelerations{};
};
struct ProgramFrame {
  double seconds{};
  std::array<std::vector<std::uint8_t>,6> payloads;
};
struct PreparedMotionProgram {
  std::string id, model_id, revision, settings_revision;
  std::array<bool,6> participating{};
  std::array<double,6> start{}, target{};
  std::vector<ProgramPoint> points;
  std::vector<ProgramFrame> frames;
  std::string mode{"streamed"};
  bool tcp_verified{true};
  double duration{};
  std::string dispatch_mode{"streamed"};
  // Nawroty wyznaczone poza IO, wspólne dla uczestniczących osi.
  std::vector<double> reversal_times;
};
struct MotionProgramStatus {
  std::string program_id, state{"idle"}, mode, message, execution_key;
  std::array<bool,6> participating{};
  bool tcp_verified{}, busy{};
  double progress{}, elapsed{}, duration{};
  std::array<double,6> start{}, target{};
  std::string dispatch_mode{"streamed"}, sync_state;
  std::uint64_t sync_cycle{};
  std::uint8_t sync_ack_mask{};
  std::int64_t sync_due_monotonic_ns{}, sync_sent_monotonic_ns{}, sync_confirmed_monotonic_ns{};
  double sync_lateness_ms{};
};
// Stan wykonawcy zmienia wyłącznie właściciel CAN, bez dodatkowego socketu.
class MotionExecutor {
 public:
  using Time=std::chrono::steady_clock::time_point;
  std::optional<PreparedMotionProgram> program;
  MotionProgramStatus status;
  std::size_t frame{}, setup_axis{}, setup_step{};
  bool waiting{}, staged{};
  Time started{}, due{}, preparation_deadline{}, settled{}, last_settle_sample{};
  Time phase_origin{}, warmup_started{};
  std::array<Time,6> checked_position{};
  Time batch_started{}, batch_deadline{}, last_sync{}, next_check{}, check_started{};
  bool checking_position{};
  std::array<double,6> last_target{};
  bool sync_waiting_confirmation{}, sync_uncertain{};
  std::array<Time,6> staged_sent{};
  std::uint8_t staged_ack_mask{};
  Time stage_origin{}, sync_sent{};
  ProgramPoint staged_reference{};
  double staged_horizon{};
};
// Kompilacja poza cyklem IO. Sampler nie alokuje pamięci.
ProgramPoint program_reference(const PreparedMotionProgram &program, double seconds);
void compile_program_reversals(PreparedMotionProgram &program);
double program_lookahead_time(const PreparedMotionProgram &program,double phase,double horizon);
// Otoczka referencji dla nieznanej chwili próbkowania między zapytaniem a RX.
std::array<double,2> program_position_bounds(const PreparedMotionProgram &program,
    unsigned axis, double begin_seconds, double end_seconds);
PreparedMotionProgram compile_motion_program(PreparedMotionProgram program,
    const std::array<AxisConfig,6> &axes, const std::array<std::uint16_t,6> &microsteps,
    double cycle_seconds);
}
