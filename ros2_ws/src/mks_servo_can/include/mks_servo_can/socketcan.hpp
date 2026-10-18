#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <chrono>

namespace mks_servo_can {

struct Frame {
  std::uint16_t address{};
  std::vector<std::uint8_t> data;
  bool error{};
  std::uint32_t error_flags{};
  std::chrono::steady_clock::time_point received_at{};
  std::int64_t kernel_utc_ns{};
};

class Transport {
 public:
  virtual ~Transport() = default;
  virtual void open(const std::string &interface) = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;
  virtual void send(std::uint16_t address, const std::vector<std::uint8_t> &data) = 0;
  virtual std::optional<Frame> receive() = 0;
};

class SocketCan : public Transport {
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
  int lock_fd_{-1};
  std::uint32_t rx_overflow_{};
};

}  // namespace mks_servo_can
