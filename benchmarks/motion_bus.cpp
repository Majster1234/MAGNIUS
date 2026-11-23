#include "harness.hpp"
#include "mks_servo_can/bus_manager.hpp"
#include <algorithm>
#include <deque>
#include <map>
#include <linux/can/error.h>
using namespace mks_servo_can;
using namespace std::chrono_literals;
namespace {
class Emulator final : public Transport {
 public:
  bool opened{false}, disconnected{false}, own_echo{false};
  std::uint16_t missing_ack{}, missing_position{};
  std::uint16_t absent_address{};
  std::array<unsigned, 7> position_requests{};
  Time now{};
  std::array<std::uint64_t, 256> counts{};
  std::array<std::int64_t, 7> positions{}, prepared{};
  std::map<std::uint16_t, std::vector<std::uint8_t>> prepared_payloads;
  std::deque<Frame> responses;
  std::vector<std::pair<std::uint16_t, std::uint8_t>> trace;
  void open(const std::string &) override { opened = true; }
  void close() override { opened = false; }
  bool is_open() const override { return opened; }
  void send(std::uint16_t address, const std::vector<std::uint8_t> &data) override {
    if (disconnected) throw std::runtime_error("USB odłączone");
    ++counts[data[0]];
    if (trace.size() < 10000) trace.emplace_back(address, data[0]);
    if (own_echo) responses.push_back({address, data, false, 0, now + 1ms});
    if (address == 0 && data[0] == 0x4b) { positions = prepared; return; }
    if (address < 1 || address > 6) return;
    if (data[0] == 0x31) ++position_requests[address];
    if (address == absent_address) return;
    std::vector<std::uint8_t> reply;
    switch (data[0]) {
      case 0x40: reply = {0x40, 0x12, 1, 0, 9, 0x20}; break;
      case 0x00: reply = {0x82, 5}; break;
      case 0x3a: reply = {0x3a, 1}; break;
      case 0x3e: reply = {0x3e, 0}; break;
      case 0xf1: reply = {0xf1, 1}; break;
      case 0x31: {
        if (missing_position == address) return;
        reply = {0x31};
        const auto p = static_cast<std::uint64_t>(positions[address]);
        for (int shift = 40; shift >= 0; shift -= 8) reply.push_back(static_cast<std::uint8_t>(p >> shift));
        break;
      }
      case 0xf5: {
        prepared_payloads[address]=data;
        const auto raw = (std::int32_t(data[4]) << 16) | (std::int32_t(data[5]) << 8) | data[6];
        prepared[address] = raw & 0x800000 ? raw - 0x1000000 : raw;
        if (missing_ack == address) return;
        reply = {0xf5, 5}; break;
      }
      default: return;
    }
    reply.push_back(checksum(address, reply));
    responses.push_back({address, reply, false, 0, now + 1ms});
  }
  std::optional<Frame> receive() override {
    if (disconnected) throw std::runtime_error("USB odłączone");
    if (responses.empty() || responses.front().received_at > now) return std::nullopt;
    auto frame = responses.front(); responses.pop_front(); return frame;
  }
};
BusConfig configured() {
  BusConfig config; config.motion_enabled = true;
  config.commissioning_verified = config.synchronization_verified = config.stop_verified = true;
  for (std::size_t i = 0; i < kAxes; ++i)
    config.axes[i] = {static_cast<std::uint16_t>(i + 1), 1, 1, 0, -2, 2, 1, 100, 0.2, 2};
  return config;
}
struct Harness {
  Emulator *bus;
  std::unique_ptr<CanBusManager> manager;
  Time now{1s};
  explicit Harness(BusConfig config = configured()) {
    auto transport = std::make_unique<Emulator>(); bus = transport.get(); bus->now = now;
    manager = std::make_unique<CanBusManager>(config, std::move(transport));
    manager->open(now); manager->start_scan();
  }
  void run(int ms) {
    for (int i = 0; i < ms; ++i) { bus->now = now; manager->tick(now); now += 1ms; }
  }
  void ready() { run(45000); if(manager->snapshot().scanning) throw std::runtime_error("scan"); }
  CommandBatch batch(std::uint64_t sequence, double position = 0.001) const {
    CommandBatch batch; batch.positions.fill(position); batch.sequence = sequence;
    batch.created = now; batch.expires = now + 19ms; batch.source = Source::trajectory; return batch;
  }
};
}

struct Cycle {
  Harness h;
  uint64_t count{};
  Cycle() { h.ready(); if(!h.manager->activate(Source::trajectory,h.now)) throw std::runtime_error("activate"); }
  void cycle(uint64_t) { if(!h.manager->submit(h.batch(++count,0))) throw std::runtime_error("submit"); h.run(20); }
  bool verify() const { return !h.manager->snapshot().fault && h.bus->counts[0x4b]>=count; }
};
int main(int argc,char **argv) { return robot_benchmarks::main(argc,argv,[]{return Cycle{};}); }
