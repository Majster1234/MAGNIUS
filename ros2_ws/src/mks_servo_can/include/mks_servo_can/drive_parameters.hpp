#pragma once
#include <cstdint>
#include <optional>
#include <vector>

namespace mks_servo_can {
// Katalog nastaw MKS SERVO42D (instrukcja CAN v1.0.9). Odczyt uniwersalny: 00h, kod[, sub];
// odpowiedź ma format zapisu (kod, parametr...) albo kod FFh FFh, gdy parametr nie obsługuje odczytu.
// writable=false: nastawa tylko zapisywana w kopii i porównywana — zapis zablokowany w C++
// (bitrate, brak weryfikacji CRC, tryb odpowiedzi i SYNC zarządzane przez sterowanie, bazowanie, autostart).
// Adres CAN (8Bh), kalibracja (80h), reset fabryczny (3Fh) i restart (41h) nie należą do katalogu.
struct DriveParameterSpec {
  std::uint8_t code;
  int sub;             // -1: brak bajtu podkomendy; 0/1: PID 96h/97h
  std::size_t length;  // bajty parametru bez kodu i sumy kontrolnej (z podkomendą)
  bool writable;
  const char *name;
};
const std::vector<DriveParameterSpec> &drive_parameter_catalog();
const DriveParameterSpec *find_drive_parameter(std::uint8_t code, int sub);
std::vector<std::uint8_t> drive_parameter_read_request(std::uint16_t address, const DriveParameterSpec &spec);
// Wyjątek std::invalid_argument: kod tylko do odczytu, zła długość albo podkomenda.
std::vector<std::uint8_t> drive_parameter_write_request(std::uint16_t address, const DriveParameterSpec &spec,
                                                        const std::vector<std::uint8_t> &value);
struct DriveParameterValue {
  bool supported{};
  std::vector<std::uint8_t> value;  // znormalizowane: zawsze z bajtem podkomendy, długość = spec.length
};
// nullopt: ramka niezgodna z nastawą (kod, suma kontrolna, długość).
std::optional<DriveParameterValue> parse_drive_parameter(std::uint16_t address, const DriveParameterSpec &spec,
                                                         const std::vector<std::uint8_t> &frame);
// Status zapisu: 1 = przyjęty, 0 = odrzucony; nullopt przy niezgodnej ramce.
std::optional<bool> parse_drive_parameter_write(std::uint16_t address, const DriveParameterSpec &spec,
                                                const std::vector<std::uint8_t> &frame);
}  // namespace mks_servo_can
