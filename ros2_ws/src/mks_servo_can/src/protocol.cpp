#include "mks_servo_can/protocol.hpp"

#include <stdexcept>

namespace mks_servo_can {
namespace {
std::optional<std::uint8_t> parse_byte(std::uint16_t address,
                                       const std::vector<std::uint8_t> &data,
                                       std::uint8_t command) {
  if (!validate_response(address, data, command, 3).valid) return std::nullopt;
  return data[1];
}
}  // namespace

std::uint8_t checksum(std::uint16_t address, const std::vector<std::uint8_t> &bytes) {
  std::uint8_t sum = static_cast<std::uint8_t>(address);
  for (const auto byte : bytes) sum = static_cast<std::uint8_t>(sum + byte);
  return sum;
}

std::vector<std::uint8_t> request(std::uint16_t address, std::uint8_t command,
                                  std::vector<std::uint8_t> payload) {
  if (address == kBroadcastAddress || address > kMaximumAddress) {
    throw std::invalid_argument("adres MKS musi należeć do zakresu 1–2047");
  }
  std::vector<std::uint8_t> bytes{command};
  bytes.insert(bytes.end(), payload.begin(), payload.end());
  bytes.push_back(checksum(address, bytes));
  return bytes;
}

std::vector<std::uint8_t> mode_request(std::uint16_t address) {
  return request(address, 0x00, {0x82});
}

ValidationResult validate_response(std::uint16_t expected_address,
                                   const std::vector<std::uint8_t> &data,
                                   std::uint8_t expected_command,
                                   std::size_t expected_length) {
  if (expected_address == kBroadcastAddress || expected_address > kMaximumAddress)
    return {false, "nieprawidłowy adres oczekiwany"};
  if (data.size() != expected_length) return {false, "nieprawidłowa długość DLC"};
  if (data.empty() || data.front() != expected_command) return {false, "nieprawidłowa komenda"};
  const std::vector<std::uint8_t> without_crc(data.begin(), data.end() - 1);
  if (data.back() != checksum(expected_address, without_crc)) return {false, "błędna suma kontrolna"};
  return {true, ""};
}

std::optional<Identity> parse_identity(std::uint16_t address,
                                       const std::vector<std::uint8_t> &data) {
  // SERVO42D CAN v1.0.9 odpowiada siedmioma bajtami: 40, typ, wersja,
  // informacje sprzętowe i suma kontrolna. Potwierdzone na magistrali.
  if (!validate_response(address, data, 0x40, 7).valid) return std::nullopt;
  const auto kind = static_cast<std::uint8_t>(data[1] & 0x0f);
  Model model = Model::kUnknown;
  if (kind == 2) model = Model::kServo42D;
  if (kind == 4) model = Model::kServo57D;
  if (kind == 6) model = Model::kServo28D;
  if (kind == 8) model = Model::kServo35D;
  return Identity{model, static_cast<std::uint8_t>(data[1] >> 4), kind, data[2], data[3]};
}

std::optional<bool> parse_enable(std::uint16_t address, const std::vector<std::uint8_t> &data) {
  const auto value = parse_byte(address, data, 0x3a);
  if (!value || *value > 1) return std::nullopt;
  return *value == 1;
}
std::optional<std::uint8_t> parse_motor_state(std::uint16_t address,
                                              const std::vector<std::uint8_t> &data) {
  const auto value = parse_byte(address, data, 0xf1);
  if (!value || *value > 6) return std::nullopt;
  return value;
}
std::optional<bool> parse_stall(std::uint16_t address, const std::vector<std::uint8_t> &data) {
  const auto value = parse_byte(address, data, 0x3e);
  if (!value || *value > 1) return std::nullopt;
  return *value == 1;
}
std::optional<std::uint8_t> parse_work_mode(std::uint16_t address,
                                            const std::vector<std::uint8_t> &data) {
  const auto value = parse_byte(address, data, 0x82);
  if (!value || *value > 5) return std::nullopt;
  return value;
}
std::string model_name(Model model) {
  switch (model) {
    case Model::kServo42D: return "MKS SERVO42D CAN";
    case Model::kServo57D: return "MKS SERVO57D CAN";
    case Model::kServo28D: return "MKS SERVO28D CAN";
    case Model::kServo35D: return "MKS SERVO35D CAN";
    default: return "Nieznany model MKS";
  }
}
std::string motor_state_name(std::uint8_t state) {
  static const char *const labels[] = {"Błąd odczytu", "Zatrzymany", "Przyspieszanie", "Hamowanie", "Pełna prędkość", "Bazowanie", "Kalibracja"};
  return state <= 6 ? labels[state] : "Niedostępny";
}
std::string work_mode_name(std::uint8_t mode) {
  static const char *const labels[] = {"CR_OPEN", "CR_CLOSE", "CR_vFOC", "SR_OPEN", "SR_CLOSE", "SR_vFOC"};
  return mode <= 5 ? labels[mode] : "Niedostępny";
}
bool supports_motor_state(std::uint8_t mode) { return mode >= 3 && mode <= 5; }
}  // namespace mks_servo_can
