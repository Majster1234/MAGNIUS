#pragma once
#include <array>
#include <cstdint>
#include <vector>
#include <optional>
#include "mks_servo_can/motion_executor.hpp"

namespace mks_servo_can {
struct AxisConfig;
struct StreamingParameters {
  double lookahead_time_sec{.085}, velocity_ff_scale{1.0}, position_kp{2.5};
  unsigned min_holding_rpm{1}, max_rpm{2500}, encoder_cpr{16384};
  std::uint8_t acceleration{135};
};
struct StreamingState {
  double rpm_dither_accum{};
  int direction{};
};
struct StreamingSetpoint {
  double lookahead_position{};
  std::int32_t ticks{};
  std::uint16_t rpm{};
  std::array<std::uint8_t,8> bytes{};
};
StreamingSetpoint streaming_setpoint(const AxisConfig &axis,double position,double velocity,
    std::uint16_t microstep=16);
// Stan jest zatwierdzany przez właściciela CAN tylko dla faktycznie wysłanego zestawu.
// Brak świeżego pomiaru wyłącza samą korekcję pozycji, bez blokowania trajektorii.
StreamingSetpoint streaming_setpoint(const AxisConfig &axis,double position,double velocity,
    StreamingState &state,std::optional<double> actual_position=std::nullopt,std::uint16_t microstep=16);
// Pełny koszt: F5 i asynchroniczne pozycje 100 Hz oraz zachowana diagnostyka.
double streaming_bus_utilization(unsigned axes,unsigned bitrate);
PreparedMotionProgram compile_streaming_program(PreparedMotionProgram program,
    const std::array<AxisConfig,6> &axes,const std::array<std::uint16_t,6> &microsteps);
}
