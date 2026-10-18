#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace mks_servo_can {

constexpr std::uint16_t kBroadcastAddress = 0;
constexpr std::uint16_t kMaximumAddress = 0x7ff;

enum class Model { kServo42D, kServo57D, kServo28D, kServo35D, kUnknown };

struct Identity {
  Model model{Model::kUnknown};
  std::uint8_t calibration{};
  std::uint8_t hardware_type{};
  std::uint8_t firmware_major{};
  std::uint8_t firmware_minor{};
};

struct ValidationResult {
  bool valid{};
  std::string error;
};

std::uint8_t checksum(std::uint16_t address, const std::vector<std::uint8_t> &bytes);
std::vector<std::uint8_t> request(std::uint16_t address, std::uint8_t command,
                                  std::vector<std::uint8_t> payload = {});
std::vector<std::uint8_t> mode_request(std::uint16_t address);
ValidationResult validate_response(std::uint16_t expected_address,
                                   const std::vector<std::uint8_t> &data,
                                   std::uint8_t expected_command,
                                   std::size_t expected_length);
std::optional<Identity> parse_identity(std::uint16_t address,
                                       const std::vector<std::uint8_t> &data);
std::optional<bool> parse_enable(std::uint16_t address,
                                 const std::vector<std::uint8_t> &data);
std::optional<std::uint8_t> parse_motor_state(std::uint16_t address,
                                              const std::vector<std::uint8_t> &data);
std::optional<bool> parse_stall(std::uint16_t address,
                                const std::vector<std::uint8_t> &data);
std::optional<std::uint8_t> parse_work_mode(std::uint16_t address,
                                            const std::vector<std::uint8_t> &data);
std::string model_name(Model model);
std::string motor_state_name(std::uint8_t state);
std::string work_mode_name(std::uint8_t mode);
bool supports_motor_state(std::uint8_t mode);

}  // namespace mks_servo_can
