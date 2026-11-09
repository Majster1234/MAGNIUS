#include "mks_servo_can/drive_parameters.hpp"
#include <stdexcept>

namespace mks_servo_can {
namespace {
std::uint8_t checksum(std::uint16_t address, const std::vector<std::uint8_t> &data) {
  unsigned sum = address;
  for (auto byte : data) sum += byte;
  return static_cast<std::uint8_t>(sum);
}
bool valid_frame(std::uint16_t address, std::uint8_t code, const std::vector<std::uint8_t> &frame) {
  if (frame.size() < 2 || frame.front() != code) return false;
  return checksum(address, std::vector<std::uint8_t>(frame.begin(), frame.end() - 1)) == frame.back();
}
}  // namespace

const std::vector<DriveParameterSpec> &drive_parameter_catalog() {
  static const std::vector<DriveParameterSpec> catalog{
      {0x82, -1, 1, true, "work_mode"},
      {0x83, -1, 2, true, "work_current_ma"},
      {0x9b, -1, 1, true, "hold_current_ratio"},
      {0x84, -1, 1, true, "microstep"},
      {0x85, -1, 1, true, "en_active_level"},
      {0x86, -1, 1, true, "direction"},
      {0x87, -1, 1, true, "auto_screen_off"},
      {0x88, -1, 1, true, "stall_protection"},
      {0x89, -1, 1, true, "subdivision_interpolation"},
      {0x8a, -1, 1, false, "can_bitrate"},
      {0x8c, -1, 2, false, "response_mode"},
      {0x8d, -1, 2, true, "group_can_id"},
      {0x8f, -1, 1, true, "key_lock"},
      {0x95, -1, 3, true, "position_reached_threshold"},
      {0x96, 0, 5, true, "vfoc_kp_ki"},
      {0x96, 1, 5, true, "vfoc_kd_kv"},
      {0x97, 0, 5, true, "close_kp_ki"},
      {0x97, 1, 5, true, "close_kd_kv"},
      {0x98, -1, 4, true, "heartbeat_ms"},
      {0x9d, -1, 5, true, "position_error_protection"},
      {0x9f, -1, 1, true, "in1_port_mode"},
      {0x42, -1, 4, true, "user_id"},
      {0x4a, -1, 1, false, "sync_mode"},
      {0x0f, -1, 1, false, "no_crc_mode"},
      {0x90, -1, 6, false, "home_parameters"},
      {0x94, -1, 6, false, "home_offset_current"},
      {0x9a, -1, 4, false, "single_turn_home"},
      {0x9e, -1, 1, false, "limit_port_mapping"},
      {0x99, -1, 5, false, "pulse_divider_output"},
      {0xff, -1, 1, false, "power_on_autorun"},
  };
  return catalog;
}

const DriveParameterSpec *find_drive_parameter(std::uint8_t code, int sub) {
  for (const auto &spec : drive_parameter_catalog())
    if (spec.code == code && spec.sub == sub) return &spec;
  return nullptr;
}

std::vector<std::uint8_t> drive_parameter_read_request(std::uint16_t address, const DriveParameterSpec &spec) {
  std::vector<std::uint8_t> data{0x00, spec.code};
  if (spec.sub >= 0) data.push_back(static_cast<std::uint8_t>(spec.sub));
  data.push_back(checksum(address, data));
  return data;
}

std::vector<std::uint8_t> drive_parameter_write_request(std::uint16_t address, const DriveParameterSpec &spec,
                                                        const std::vector<std::uint8_t> &value) {
  if (!spec.writable) throw std::invalid_argument("Nastawa tylko do odczytu");
  if (value.size() != spec.length) throw std::invalid_argument("Nieprawidłowa długość nastawy");
  if (spec.sub >= 0 && value.front() != spec.sub) throw std::invalid_argument("Nieprawidłowa podkomenda nastawy");
  std::vector<std::uint8_t> data{spec.code};
  data.insert(data.end(), value.begin(), value.end());
  data.push_back(checksum(address, data));
  return data;
}

std::optional<DriveParameterValue> parse_drive_parameter(std::uint16_t address, const DriveParameterSpec &spec,
                                                         const std::vector<std::uint8_t> &frame) {
  if (!valid_frame(address, spec.code, frame)) return std::nullopt;
  std::vector<std::uint8_t> value(frame.begin() + 1, frame.end() - 1);
  if (value.size() == 2 && value[0] == 0xff && value[1] == 0xff) return DriveParameterValue{false, {}};
  // PID: odpowiedź z bajtem podkomendy (DLC7) albo bez niego (DLC6) — normalizacja do formatu zapisu.
  if (spec.sub >= 0 && value.size() + 1 == spec.length) value.insert(value.begin(), static_cast<std::uint8_t>(spec.sub));
  if (spec.sub >= 0 && (value.empty() || value.front() != spec.sub)) return std::nullopt;
  // 83h: część firmware dołącza bajt 00h po wartości prądu.
  if (spec.code == 0x83 && value.size() == 3 && value[2] == 0) value.pop_back();
  if (value.size() != spec.length) return std::nullopt;
  return DriveParameterValue{true, value};
}

std::optional<bool> parse_drive_parameter_write(std::uint16_t address, const DriveParameterSpec &spec,
                                                const std::vector<std::uint8_t> &frame) {
  if (frame.size() != 3 || !valid_frame(address, spec.code, frame) || frame[1] > 1) return std::nullopt;
  return frame[1] == 1;
}
}  // namespace mks_servo_can
