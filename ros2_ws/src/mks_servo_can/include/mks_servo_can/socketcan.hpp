#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace mks_servo_can {

struct Frame {
  std::uint16_t address{};
  std::vector<std::uint8_t> data;
  bool error{};
  std::uint32_t error_flags{};
};

class SocketCan {
 public:
  SocketCan() = default;
  ~SocketCan();
  SocketCan(const SocketCan &) = delete;
  SocketCan &operator=(const SocketCan &) = delete;

  void open(const std::string &interface);
  void close();
  bool is_open() const;
  void send(std::uint16_t address, const std::vector<std::uint8_t> &data);
  std::optional<Frame> receive();

 private:
  int fd_{-1};
};

}  // namespace mks_servo_can
