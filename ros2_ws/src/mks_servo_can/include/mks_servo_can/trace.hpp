#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace mks_servo_can {
// Stały rozmiar zdarzenia i kolejki MPSC. Producent nie czeka na dysk ani mutex.
struct TraceEvent {
  const char *event{"unknown"};
  std::int64_t mono_ns{}, utc_ns{}, request_ns{}, deadline_ns{}, duration_ns{}, kernel_utc_ns{};
  std::int64_t position_age_ns{-1}, speed_age_ns{-1}, enable_age_ns{-1}, stall_age_ns{-1};
  std::uint64_t sequence{};
  unsigned axis{}, command{}, flags{}, dlc{};
  std::array<std::uint8_t, 8> data{};
  std::array<char, 192> detail{};
};
// Zdarzenia awarii: po ich zapisaniu na dysk wątek zapisu tworzy faults/trigger-*.json dla migawki awarii
// (robot_web_bridge/fault_snapshot.py). Nagranie ciągłe nie zatrzymuje się po awarii.
bool is_fault_trace_event(const char *event) noexcept;
class TraceRecorder {
 public:
  // Pętla segments plików can-0…can-(segments−1).jsonl po segment_bytes; najstarszy segment jest nadpisywany.
  explicit TraceRecorder(const std::string &directory, std::size_t segment_bytes = 16 * 1024 * 1024, unsigned segments = 4);
  ~TraceRecorder();
  bool submit(TraceEvent event) noexcept;
  std::uint64_t dropped() const { return dropped_; }
 private:
  void write_loop();
  void write(const TraceEvent &event);
  void write_fault_trigger(const TraceEvent &event) noexcept;
  std::string directory_;
  std::string recording_id_;
  std::size_t max_bytes_, written_{};
  unsigned segments_, slot_{};
  std::ofstream file_;
  struct Cell { std::atomic<bool> ready{false}; TraceEvent event; };
  std::array<Cell, 4096> queue_{};
  std::atomic<std::uint64_t> head_{0}, tail_{0};
  std::atomic<unsigned> producers_{0};
  std::mutex mutex_;
  std::condition_variable wake_;
  std::atomic<bool> running_{true};
  std::atomic<std::uint64_t> dropped_{0}, sequence_{0};
  std::thread worker_;
};
std::int64_t trace_ns(std::chrono::steady_clock::time_point time);
void trace(TraceEvent event) noexcept;
void trace_note(const char *event, const std::string &detail = "", unsigned axis = 0, unsigned command = 0) noexcept;
}  // namespace mks_servo_can
