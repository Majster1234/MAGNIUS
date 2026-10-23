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
};
struct MotionProgramStatus {
  std::string program_id, state{"idle"}, mode, message, execution_key;
  std::array<bool,6> participating{};
  bool tcp_verified{}, busy{};
  double progress{}, elapsed{}, duration{};
  std::array<double,6> start{}, target{};
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
};
// Kompilacja poza cyklem IO. Sampler nie alokuje pamięci.
ProgramPoint program_reference(const PreparedMotionProgram &program, double seconds);
// Otoczka referencji dla nieznanej chwili próbkowania między zapytaniem a RX.
std::array<double,2> program_position_bounds(const PreparedMotionProgram &program,
    unsigned axis, double begin_seconds, double end_seconds);
PreparedMotionProgram compile_motion_program(PreparedMotionProgram program,
    const std::array<AxisConfig,6> &axes, const std::array<std::uint16_t,6> &microsteps,
    double cycle_seconds);
}
