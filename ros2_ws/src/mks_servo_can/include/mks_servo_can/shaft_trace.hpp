#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <cstddef>
#define MKS_SHAFT_TRACE_AVAILABLE 1
namespace mks_servo_can {
// flags: bit 0 nowy segment (zmiana zera), kShaftReferenceFlag — pozycja referencyjna ruchu zamiast pomiaru 0x31,
// kShaftCommandFlag — cel F5 faktycznie wysłany do napędu (impulsy wału), kShaftRpmFlag — prędkość F5 wału w 0,01 RPM.
inline constexpr std::uint32_t kShaftReferenceFlag=0x100;
inline constexpr std::uint32_t kShaftCommandFlag=0x200;
inline constexpr std::uint32_t kShaftRpmFlag=0x400;
struct ShaftPositionSample {
  std::uint64_t sequence{}, segment{}, monotonic_ns{}, utc_ns{};
  std::int64_t ticks{};
  std::uint32_t flags{};
  std::uint8_t axis{};
};
// Jeden producent (IO) i jeden konsument (publikacja ROS). Bez alokacji i mutexu.
class ShaftTraceQueue {
 public:
  static constexpr std::size_t capacity=4096;
  bool push(const ShaftPositionSample &sample) noexcept {
    const auto tail=tail_.load(std::memory_order_relaxed);
    if(tail-head_.load(std::memory_order_acquire)>=capacity) { ++dropped_;return false; }
    entries_[tail%capacity]=sample;tail_.store(tail+1,std::memory_order_release);return true;
  }
  std::size_t drain(ShaftPositionSample *out,std::size_t maximum) noexcept {
    auto head=head_.load(std::memory_order_relaxed);const auto tail=tail_.load(std::memory_order_acquire);
    std::size_t n=0;while(n<maximum && head<tail) out[n++]=entries_[head++%capacity];
    head_.store(head,std::memory_order_release);return n;
  }
  std::uint64_t dropped() const noexcept { return dropped_.load(); }
 private:
  std::array<ShaftPositionSample,capacity> entries_{};
  alignas(64) std::atomic<std::uint64_t> head_{0};
  alignas(64) std::atomic<std::uint64_t> tail_{0};
  std::atomic<std::uint64_t> dropped_{0};
};
}
