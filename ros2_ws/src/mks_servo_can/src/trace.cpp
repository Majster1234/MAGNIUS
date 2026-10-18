#include "mks_servo_can/trace.hpp"
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

namespace mks_servo_can {
namespace {
std::string quote(const char *s) {
  std::ostringstream out; out << '"';
  for (const auto *p = reinterpret_cast<const unsigned char *>(s); *p; ++p) {
    if (*p == '"' || *p == '\\') out << '\\' << *p;
    else if (*p < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(*p) << std::dec;
    else out << *p;
  }
  out << '"'; return out.str();
}
TraceRecorder *recorder() noexcept {
  static const auto instance = []() -> std::unique_ptr<TraceRecorder> {
    const auto *dir = std::getenv("ROBOT_TRACE_DIR");
    if (!dir || !*dir) return {};
    try { return std::make_unique<TraceRecorder>(dir); }
    catch (const std::exception &e) { std::cerr << "Zapis diagnostyczny CAN niedostępny: " << e.what() << '\n'; return {}; }
  }();
  return instance.get();
}
}
std::int64_t trace_ns(std::chrono::steady_clock::time_point time) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}
TraceRecorder::TraceRecorder(const std::string &directory, std::size_t max_bytes)
    : directory_(directory), max_bytes_(max_bytes) {
  if (const auto *id = std::getenv("ROBOT_TRACE_SESSION")) recording_id_ = std::string(id).substr(0, 128);
  if (max_bytes < 2048) throw std::invalid_argument("Za mały limit pliku diagnostycznego");
  std::filesystem::create_directories(directory_);
  file_.open(directory_ + "/can-0.jsonl", std::ios::trunc);
  if (!file_) throw std::runtime_error("Nie można otworzyć pliku diagnostycznego CAN");
  worker_ = std::thread([this] { write_loop(); });
}
TraceRecorder::~TraceRecorder() {
  running_ = false; wake_.notify_all(); if (worker_.joinable()) worker_.join();
}
void TraceRecorder::freeze_after(std::chrono::milliseconds delay) noexcept {
  std::int64_t empty = 0;
  freeze_at_.compare_exchange_strong(empty, trace_ns(std::chrono::steady_clock::now() + delay));
}
bool TraceRecorder::submit(TraceEvent event) noexcept {
  struct ProducerGuard { std::atomic<unsigned> &count; ~ProducerGuard() { --count; } };
  ++producers_; ProducerGuard guard{producers_};
  if (!running_) return false;
  event.sequence = ++sequence_;
  if (!event.mono_ns) event.mono_ns = trace_ns(std::chrono::steady_clock::now());
  if (!event.utc_ns) event.utc_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  auto tail = tail_.load(std::memory_order_relaxed);
  for (unsigned attempt = 0; attempt < 8; ++attempt) {
    const auto head = head_.load(std::memory_order_acquire);
    if (tail < head) { tail = tail_.load(std::memory_order_relaxed); continue; }
    if (tail - head >= queue_.size()) break;
    if (!tail_.compare_exchange_weak(tail, tail + 1, std::memory_order_relaxed)) continue;
    auto &cell = queue_[tail % queue_.size()]; cell.event = event;
    cell.ready.store(true, std::memory_order_release);
    if (tail + 1 - head >= 128 || !running_) wake_.notify_one();
    return true;
  }
  ++dropped_; return false;
}
void TraceRecorder::write_loop() {
  std::array<TraceEvent, 128> batch;
  while (true) {
    if (freeze_at_ && trace_ns(std::chrono::steady_clock::now()) >= freeze_at_) running_ = false;
    std::size_t size = 0;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait_for(lock, std::chrono::milliseconds(100), [this] { return tail_.load() - head_.load() >= 128 || !running_; });
    }
    auto head = head_.load(std::memory_order_relaxed);
    while (size < batch.size() && head < tail_.load(std::memory_order_acquire)) {
      auto &cell = queue_[head % queue_.size()];
      if (!cell.ready.load(std::memory_order_acquire)) break;
      batch[size++] = cell.event; cell.ready.store(false, std::memory_order_relaxed);
      head_.store(++head, std::memory_order_release);
    }
    if (!size && !running_ && head == tail_.load() && !producers_) {
      TraceEvent end; end.event = "recording_finished"; end.sequence = ++sequence_;
      end.mono_ns = trace_ns(std::chrono::steady_clock::now());
      end.utc_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      try { write(end); file_.flush(); } catch (const std::exception &) { /* Logger nie zatrzymuje aplikacji. */ }
      break;
    }
    try {
      for (std::size_t i = 0; i < size; ++i) write(batch[i]);
      file_.flush();
      if (!file_) throw std::runtime_error("Błąd zapisu pliku diagnostycznego");
    } catch (const std::exception &e) {
      std::cerr << "Zatrzymano zapis diagnostyczny CAN: " << e.what() << '\n'; running_ = false; break;
    }
  }
}
void TraceRecorder::write(const TraceEvent &e) {
  std::ostringstream out;
  out << "{\"source\":\"can_cpp\",\"pid\":" << ::getpid() << ",\"event\":" << quote(e.event)
      << ",\"recording_id\":" << quote(recording_id_.c_str())
      << ",\"sequence\":" << e.sequence << ",\"mono_ns\":" << e.mono_ns << ",\"utc_ns\":" << e.utc_ns
      << ",\"axis\":" << e.axis << ",\"command\":" << e.command << ",\"flags\":" << e.flags
      << ",\"dropped_total\":" << dropped_.load();
  if (e.request_ns) out << ",\"request_ns\":" << e.request_ns;
  if (e.deadline_ns) out << ",\"deadline_ns\":" << e.deadline_ns;
  if (e.duration_ns) out << ",\"duration_ns\":" << e.duration_ns;
  if (e.kernel_utc_ns) out << ",\"kernel_utc_ns\":" << e.kernel_utc_ns;
  if (e.position_age_ns >= 0) out << ",\"position_age_ns\":" << e.position_age_ns;
  if (e.speed_age_ns >= 0) out << ",\"speed_age_ns\":" << e.speed_age_ns;
  if (e.enable_age_ns >= 0) out << ",\"enable_age_ns\":" << e.enable_age_ns;
  if (e.stall_age_ns >= 0) out << ",\"stall_age_ns\":" << e.stall_age_ns;
  out << ",\"data\":\"";
  for (unsigned i = 0; i < std::min(e.dlc, 8u); ++i) out << std::hex << std::setw(2) << std::setfill('0') << unsigned(e.data[i]);
  out << "\",\"detail\":" << quote(e.detail.data()) << "}\n";
  const auto line = out.str();
  if (written_ + line.size() > max_bytes_) {
    file_.close(); slot_ = (slot_ + 1) % 4; written_ = 0;
    file_.open(directory_ + "/can-" + std::to_string(slot_) + ".jsonl", std::ios::trunc);
  }
  file_ << line; written_ += line.size();
}
void trace(TraceEvent event) noexcept { if (auto *r = recorder()) r->submit(event); }
void trace_freeze_after(std::chrono::milliseconds delay) noexcept { if (auto *r = recorder()) r->freeze_after(delay); }
void trace_note(const char *event, const std::string &detail, unsigned axis, unsigned command) noexcept {
  TraceEvent e; e.event = event; e.axis = axis; e.command = command;
  std::copy_n(detail.data(), std::min(detail.size(), e.detail.size() - 1), e.detail.data()); trace(e);
}
}  // namespace mks_servo_can
