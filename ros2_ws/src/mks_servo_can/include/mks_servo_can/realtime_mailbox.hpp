#pragma once
#include "mks_servo_can/bus_manager.hpp"
#include <atomic>

namespace mks_servo_can {
// Single-writer/latest-value: brak kolejki zaległych punktów i brak mutexu RT.
class CommandMailbox {
 public:
  CommandMailbox() {for(unsigned j=0;j<kAxes;++j) {positions_[j]=0;velocities_[j]=0;}}
  void publish(const CommandBatch &batch) {
    version_.fetch_add(1);
    for(unsigned j=0;j<kAxes;++j) {positions_[j].store(batch.positions[j]);velocities_[j].store(batch.velocities[j]);}
    created_=std::chrono::duration_cast<std::chrono::nanoseconds>(batch.created.time_since_epoch()).count();
    expires_=std::chrono::duration_cast<std::chrono::nanoseconds>(batch.expires.time_since_epoch()).count();
    sequence_=batch.sequence;generation_=batch.configuration_generation;source_=static_cast<unsigned>(batch.source);velocities_valid_=batch.velocities_valid;
    version_.fetch_add(1);
  }
  bool take(CommandBatch &batch,std::uint64_t &seen) const {
    const auto version=version_.load();if(!version || version==seen || (version&1)) return false;
    for(unsigned j=0;j<kAxes;++j) {batch.positions[j]=positions_[j].load();batch.velocities[j]=velocities_[j].load();}
    batch.created=Time(std::chrono::duration_cast<Clock::duration>(std::chrono::nanoseconds(created_.load())));
    batch.expires=Time(std::chrono::duration_cast<Clock::duration>(std::chrono::nanoseconds(expires_.load())));
    batch.sequence=sequence_.load();batch.configuration_generation=generation_.load();batch.source=static_cast<Source>(source_.load());batch.velocities_valid=velocities_valid_.load();
    if(version!=version_.load()) return false;
    seen=version;return true;
  }
 private:
  std::atomic<std::uint64_t> version_{0},sequence_{0},generation_{0};
  std::atomic<std::int64_t> created_{0},expires_{0};
  std::atomic<unsigned> source_{0};
  std::atomic<bool> velocities_valid_{false};
  std::array<std::atomic<double>,kAxes> positions_{},velocities_{};
};
class ControlMailbox {
 public:
  ControlMailbox() {for(unsigned j=0;j<kAxes;++j) {positions_[j]=0;stamps_[j]=0;}}
  void publish(const ControlState &state) {
    version_.fetch_add(1);
    for(unsigned j=0;j<kAxes;++j) {
      positions_[j].store(state.positions[j].value);
      stamps_[j].store(state.positions[j].valid ? std::chrono::duration_cast<std::chrono::nanoseconds>(state.positions[j].stamp.time_since_epoch()).count() : 0);
      const auto &c=state.conversions[j];ratios_[j]=c.gear_ratio;directions_[j]=c.direction;offsets_[j]=c.offset;
      addresses_[j]=c.address;calibrated_[j]=c.calibrated;
    }
    generation_=state.configuration_generation;
    flags_=unsigned(state.active) | (unsigned(state.fault)<<1) | (unsigned(state.program_owned)<<2) | (unsigned(state.configuration_ready)<<3) | (unsigned(state.source)<<4);
    version_.fetch_add(1);
  }
  bool read(ControlState &state) const {
    const auto version=version_.load();if(!version || (version&1)) return false;
    ControlState next;
    for(unsigned j=0;j<kAxes;++j) {
      const auto stamp=stamps_[j].load();
      next.positions[j]={positions_[j].load(),Time(std::chrono::duration_cast<Clock::duration>(std::chrono::nanoseconds(stamp))),stamp!=0};
      next.conversions[j]={ratios_[j].load(),directions_[j].load(),offsets_[j].load(),addresses_[j].load(),calibrated_[j].load()};
    }
    next.configuration_generation=generation_.load();
    const auto flags=flags_.load();next.active=flags&1;next.fault=flags&2;next.program_owned=flags&4;next.configuration_ready=flags&8;next.source=static_cast<Source>(flags>>4);
    if(version!=version_.load()) return false;
    state=next;return true;
  }
 private:
  std::atomic<std::uint64_t> version_{0};
  std::atomic<std::uint64_t> generation_{0};
  std::atomic<unsigned> flags_{0};
  std::array<std::atomic<double>,kAxes> positions_{};
  std::array<std::atomic<double>,kAxes> ratios_{},directions_{},offsets_{};
  std::array<std::atomic<std::uint16_t>,kAxes> addresses_{};
  std::array<std::atomic<bool>,kAxes> calibrated_{};
  std::array<std::atomic<std::int64_t>,kAxes> stamps_{};
};
static_assert(std::atomic<std::int64_t>::is_always_lock_free && std::atomic<double>::is_always_lock_free,
    "Platforma RT wymaga lock-free atomics 64-bit");
}
