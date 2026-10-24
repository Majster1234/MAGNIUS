#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <chrono>
#include <atomic>
#include <thread>

namespace mks_servo_can {

struct Frame {
  std::uint16_t address{};
  std::vector<std::uint8_t> data;
  bool error{};
  std::uint32_t error_flags{};
  std::chrono::steady_clock::time_point received_at{};
  std::int64_t kernel_utc_ns{};
};
struct AxisFeedback {
  std::int64_t pulses{};
  std::uint64_t timestamp_ns{};
  bool valid{};
};
class FrameQueue {
 public:
  bool push(Frame frame) {
    const auto head=head_.load(std::memory_order_relaxed),next=(head+1)%frames_.size();
    if(next==tail_.load(std::memory_order_acquire)) return false;
    frames_[head]=std::move(frame);head_.store(next,std::memory_order_release);return true;
  }
  std::optional<Frame> pop() {
    const auto tail=tail_.load(std::memory_order_relaxed);
    if(tail==head_.load(std::memory_order_acquire)) return {};
    Frame frame=std::move(frames_[tail]);tail_.store((tail+1)%frames_.size(),std::memory_order_release);return frame;
  }
  void reset() { head_=0;tail_=0; }
 private:
  std::array<Frame,4096> frames_{};
  std::atomic<std::size_t> head_{0},tail_{0};
};

class Transport {
 public:
  virtual ~Transport() = default;
  virtual void open(const std::string &interface) = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;
  virtual void send(std::uint16_t address, const std::vector<std::uint8_t> &data) = 0;
  virtual std::optional<Frame> receive() = 0;
  virtual AxisFeedback position_feedback(unsigned) const { return {}; }
  virtual void set_receive_poll_timeout(unsigned) {}
  virtual unsigned bitrate() const { return 0; } // 0: transport emulowany, bez zegara przewodu
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
  AxisFeedback position_feedback(unsigned address) const override;
  void set_receive_poll_timeout(unsigned timeout_ms) override;
  unsigned bitrate() const override { return link_bitrate_; }

 private:
  friend struct SocketCanTestAccess;
  int fd_{-1};
  int lock_fd_{-1};
  std::uint32_t rx_overflow_{};
  std::atomic<bool> receiving_{false};
  std::atomic<int> receive_error_{0};
  unsigned poll_timeout_ms_{1};
  unsigned link_bitrate_{};
  std::thread receiver_;
  FrameQueue frames_;
  struct AtomicAxis {
    std::atomic<std::uint64_t> sequence{0},timestamp_ns{0};
    std::atomic<std::int64_t> pulses{0};
  };
  std::array<AtomicAxis,2048> positions_{};
  void receive_loop();
  std::optional<Frame> receive_socket();
};

}  // namespace mks_servo_can
