#include "mks_servo_can/protocol.hpp"

#include <stdexcept>

namespace mks_servo_can {
std::vector<std::uint8_t> speed_request(std::uint16_t address, bool clockwise,
                                      std::uint16_t rpm, std::uint8_t acceleration) {
  if (rpm > 3000 || acceleration == 0) throw std::invalid_argument("Nieprawidłowa prędkość/rampa JOG");
  return request(address, 0xf6, {static_cast<std::uint8_t>((clockwise ? 0x80 : 0) | (rpm >> 8)),
                                static_cast<std::uint8_t>(rpm), acceleration});
}
std::vector<std::uint8_t> enable_request(std::uint16_t address, bool enabled) {
  return request(address, 0xf3, {static_cast<std::uint8_t>(enabled)});
}
std::vector<std::uint8_t> zero_request(std::uint16_t address) { return request(address, 0x92); }
std::vector<std::uint8_t> encoder_position_request(std::uint16_t address) { return request(address,0x31); }
std::vector<std::uint8_t> parameter_request(std::uint16_t address, std::uint8_t parameter) {
  return request(address, 0, {parameter});
}
std::optional<std::int16_t> parse_speed(std::uint16_t address, const std::vector<std::uint8_t> &data) {
  if (!validate_response(address, data, 0x32, 4).valid) return std::nullopt;
  const auto raw = static_cast<std::uint16_t>((std::uint16_t(data[1]) << 8) | data[2]);
  return static_cast<std::int16_t>(raw >= 0x8000 ? static_cast<std::int32_t>(raw) - 65536 : raw);
}
double speed_scale(std::uint16_t microstep) {
  if (!microstep || microstep > 256 || (microstep & (microstep - 1)))
    throw std::invalid_argument("Nieobsługiwany mikrokrok — wymagane 1, 2, 4, 8, 16, 32, 64, 128 lub 256");
  return microstep == 16 || microstep == 32 || microstep == 64 ? 1.0 : 16.0 / microstep;
}
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
  std::vector<std::uint8_t> bytes;bytes.reserve(payload.size()+2);bytes.push_back(command);
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
  unsigned sum=expected_address;
  for(std::size_t i=0;i+1<data.size();++i) sum+=data[i];
  if (data.back() != static_cast<std::uint8_t>(sum)) return {false, "błędna suma kontrolna"};
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

std::optional<std::int64_t> parse_position(std::uint16_t address,
                                        const std::vector<std::uint8_t> &data) {
  if (!validate_response(address, data, 0x31, 8).valid) return std::nullopt;
  std::int64_t value = 0;
  for (std::size_t i = 1; i < 7; ++i) value = (value << 8) | data[i];
  if (value & (std::int64_t{1} << 47)) value -= std::int64_t{1} << 48;
  return value;
}
std::vector<std::uint8_t> absolute_position_request(std::uint16_t address, std::int32_t ticks,
                                                 std::uint16_t rpm, std::uint8_t acceleration) {
  if (ticks < -8388608 || ticks > 8388607 || rpm == 0 || rpm > 3000)
    throw std::invalid_argument("pozycja/prędkość poza zakresem MKS F5");
  const auto bits = static_cast<std::uint32_t>(ticks);
  return request(address, 0xf5, {static_cast<std::uint8_t>((rpm >> 8)&0x0f), static_cast<std::uint8_t>(rpm),
      acceleration, static_cast<std::uint8_t>(bits >> 16), static_cast<std::uint8_t>(bits >> 8),
      static_cast<std::uint8_t>(bits)});
}
std::vector<std::uint8_t> synchronize_request() { return {0x4b, 0x4b}; }
std::vector<std::uint8_t> stop_request(std::uint16_t address) { return request(address, 0xf7); }
}  // namespace mks_servo_can
