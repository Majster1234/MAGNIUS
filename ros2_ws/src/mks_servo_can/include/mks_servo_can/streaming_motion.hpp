#pragma once
#include <array>
#include <cstdint>
#include <vector>
#include <optional>
#include "mks_servo_can/motion_executor.hpp"
#include "mks_servo_can/motion_timing.hpp"

namespace mks_servo_can {
struct AxisConfig;
struct StreamingParameters {
  double lookahead_time_sec{.085}, velocity_ff_scale{1.0}, position_kp{2.5};
  // Minimalna niezerowa prędkość F5 [RPM wału] = dolna granica ditheringu (F5 speed=0 oznacza STOP).
  // Efektywnie zaokrąglana w górę do jednostki MKS: 1 RPM przy mikrokroku 16, 0,125 RPM przy 128.
  // PTP/LIN i JOG TCP biorą ją z konfiguracji robota (`motion.min_dither_rpm`).
  double min_holding_rpm{1.};
  unsigned max_rpm{2500}, encoder_cpr{16384};
  std::uint8_t acceleration{135};
  // Człon całkujący celu F5 (opcja konfiguracji robota `motion.position_integral`): w trakcie ruchu
  // integral += gain·uchyb·10 ms, ograniczony do ±limit [rad przegubu], doliczany do celu F5.
  bool position_integral{false};
  double integral_gain{2.0}, integral_limit{0.017453292519943295};
  // Wyprzedzenie prędkości F5 (programy LIN): pole prędkości = |v + skala·a·t|, t = wyprzedzenie celu (0 = wyłączone, 1 = prędkość z chwili celu).
  double acceleration_ff_scale{0.};
  // Minimalne wyprzedzenie celu F5 przed referencją w kierunku ruchu [° wału] (test 19.11.2026; 0 = wyłączone): przy małych
  // prędkościach v·t jest mniejsze niż wahanie napędu (±1–2° wału), napęd dochodzi do celu i staje (schodki). Nie dalej niż
  // punkt zatrzymania planu przy hamowaniu i koniec trajektorii; bez wpływu na JOG TCP (jawny cel) i dolot.
  double min_lead_shaft_deg{0.};
};
struct StreamingState {
  double rpm_dither_accum{};
  int direction{};
  double integral{};
  int motion_direction{};   // znak ostatniej niezerowej prędkości planu (anty-windup członu całkującego)
};
struct StreamingSetpoint {
  double lookahead_position{};
  std::int32_t ticks{};
  std::uint16_t rpm{};
  std::array<std::uint8_t,8> bytes{};
};
StreamingSetpoint streaming_setpoint(const AxisConfig &axis,double position,double velocity,
    std::uint16_t microstep=16);
// Cel F5 z minimalnym wyprzedzeniem `streaming.min_lead_shaft_deg` przed referencją q w kierunku v (bez zmian, gdy 0, postój,
// dolot lub cel już dalej); najwyżej do punktu zatrzymania planu przy hamowaniu i do końca trajektorii. Wspólne dla strumienia
// i trybu synchronicznego (jawny cel z referencji planu); JOG TCP go nie używa.
double minimum_lead_target(const AxisConfig &axis,double position,double velocity,double reference_acceleration,
    double remaining_time_sec,double target);
// Stan jest zatwierdzany przez właściciela CAN tylko dla faktycznie wysłanego zestawu.
// Brak świeżego pomiaru wyłącza samą korekcję pozycji, bez blokowania trajektorii.
StreamingSetpoint streaming_setpoint(const AxisConfig &axis,double position,double velocity,
    StreamingState &state,std::optional<double> actual_position=std::nullopt,std::uint16_t microstep=16);
StreamingSetpoint streaming_setpoint(const AxisConfig &axis,double position,double velocity,
    StreamingState &state,std::optional<double> actual_position,std::uint16_t microstep,
    double reference_acceleration,double remaining_time_sec,double actual_age_sec=0.0,
    std::optional<double> lookahead_position=std::nullopt);
// Pełny koszt: F5 i asynchroniczne pozycje 100 Hz oraz zachowana diagnostyka.
double streaming_bus_utilization(unsigned axes,unsigned bitrate);
PreparedMotionProgram compile_streaming_program(PreparedMotionProgram program,
    const std::array<AxisConfig,6> &axes,const std::array<std::uint16_t,6> &microsteps);
}
