#include <gtest/gtest.h>
#include <algorithm>
#include <deque>
#include <map>
#include <fstream>
#include <unistd.h>
#include <linux/can/error.h>
#include "mks_servo_can/bus_manager.hpp"

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
      case 0x00:
        if(data[1]==0x8c) reply={0x8c,0,0};
        else reply={data[1],static_cast<uint8_t>(data[1]==0x82 ? 5 : data[1]==0x84 ? 16 : 0)};
        break;
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
        positions[address]=prepared[address];return; // CanRSP=Disable
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
  BusConfig config; config.bitrate=1000000;config.motion_enabled = true;
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
    manager->open(now);manager->require_configuration();RobotProfile profile;profile.revision="unit";profile.changed.fill(true);
    for(unsigned j=0;j<6;++j) {const auto &a=config.axes[j];profile.axes[j]={true,a.gear_ratio,a.direction,a.offset,a.lower,a.upper,a.max_velocity,a.max_acceleration};}
    if(!manager->configure(ConfigurationPhase::bootstrap,"boot","",profile,now).accepted || !manager->configure(ConfigurationPhase::finish,"boot","",profile,now).accepted) throw std::runtime_error("bootstrap");
  }
  void run(int ms) {
    for (int i = 0; i < ms; ++i) { bus->now = now; manager->tick(now); now += 1ms; }
  }
  void ready() { run(45000); ASSERT_FALSE(manager->snapshot().scanning); }
  CommandBatch batch(std::uint64_t sequence, double position = 0.001) const {
    CommandBatch batch;batch.velocities_valid=true; batch.positions.fill(position); batch.sequence = sequence;
    batch.created = now; batch.expires = now + 19ms; batch.source = Source::trajectory; return batch;
  }
};
}

TEST(BusManager, CompleteScanDetectsOnlyResponsesAndDoesNotMove) {
  Harness h; h.bus->own_echo = true; h.ready();
  const auto snapshot = h.manager->snapshot();
  EXPECT_EQ(snapshot.checked, 20u);
  EXPECT_EQ(std::count_if(snapshot.devices.begin(), snapshot.devices.end(), [](const auto &d) { return d.detected; }), 6);
  EXPECT_EQ(h.bus->counts[0xf5], 0u); EXPECT_EQ(h.bus->counts[0x4b], 0u);
  EXPECT_TRUE(snapshot.devices[6].position.valid);
  EXPECT_LT(snapshot.devices[6].rtt.maximum, 2.0);
}
TEST(BusManager, ShaftTraceContainsEachAcceptedEncoderAndDoesNotDuplicateCache) {
  Harness h;h.ready();std::array<ShaftPositionSample,4096> out;
  h.manager->drain_shaft_trace(out.data(),out.size());
  const auto before=h.bus->position_requests;
  for(unsigned axis=1;axis<=6;++axis) h.bus->positions[axis]=-123*static_cast<std::int64_t>(axis);
  h.run(400);
  const auto count=h.manager->drain_shaft_trace(out.data(),out.size());
  std::array<unsigned,7> received{};std::array<std::uint64_t,7> last{};
  for(unsigned i=0;i<count;++i) {
    const auto &p=out[i];ASSERT_GE(p.axis,1);ASSERT_LE(p.axis,6);
    EXPECT_EQ(p.ticks,-123*p.axis);EXPECT_GT(p.sequence,last[p.axis]);last[p.axis]=p.sequence;
    EXPECT_GT(p.segment,0u);EXPECT_GT(p.utc_ns,0u);EXPECT_GT(p.monotonic_ns,0u);++received[p.axis];
  }
  for(unsigned axis=1;axis<=6;++axis) EXPECT_EQ(received[axis],h.bus->position_requests[axis]-before[axis]);
  EXPECT_EQ(count,60u);EXPECT_EQ(h.manager->drain_shaft_trace(out.data(),out.size()),0u);
  const auto segment=out[0].segment;
  h.manager->close();h.manager->reset_session(ResetMode::preserve_discovery);h.manager->open(h.now);h.run(80);
  const auto n=h.manager->drain_shaft_trace(out.data(),out.size());ASSERT_GT(n,0u);
  EXPECT_GT(out[0].segment,segment);EXPECT_NE(out[0].flags&1,0u);
  EXPECT_EQ(h.bus->counts[0xf5],0u);EXPECT_EQ(h.bus->counts[0x4b],0u);
}
TEST(BusManager, AllSixAxesAreSentWithoutAckOrSync) {
  Harness h; h.ready(); ASSERT_TRUE(h.manager->activate(Source::trajectory, h.now));
  ASSERT_TRUE(h.manager->submit(h.batch(1))); h.run(10);
  EXPECT_EQ(h.bus->counts[0xf5], 6u); EXPECT_EQ(h.bus->counts[0x4b], 0u);
  EXPECT_EQ(h.manager->snapshot().statistics.committed, 1u);
  EXPECT_FALSE(h.manager->start_scan()); EXPECT_FALSE(h.manager->service_allowed());
}
TEST(BusManager, ProfileVelocityReachesAllDrivesAndInvalidVelocityBlocksSync) {
  Harness h; h.ready(); ASSERT_TRUE(h.manager->activate(Source::trajectory,h.now));
  auto command=h.batch(1); command.velocities_valid=true; command.velocities.fill(.15);
  ASSERT_TRUE(h.manager->submit(command)); h.run(10);
  ASSERT_EQ(h.bus->counts[0xf5],6u);
  for(unsigned i=1;i<=6;++i) {
    const auto &frame=h.bus->prepared_payloads.at(i);
    EXPECT_EQ((frame[1]<<8)|frame[2],1);  // Pierwszy krok ditheringu dla 0,15 rad/s = 1,43 RPM.
  }
  command=h.batch(2); command.velocities_valid=true; command.velocities.fill(100);
  ASSERT_TRUE(h.manager->submit(command)); h.run(20);
  EXPECT_TRUE(h.manager->snapshot().fault); EXPECT_EQ(h.bus->counts[0x4b],0u);
}
TEST(BusManager, MissingAckHasNoEffectOnFireAndForgetStream) {
  Harness h;h.ready();h.bus->missing_ack=2;
  ASSERT_TRUE(h.manager->activate(Source::trajectory,h.now));
  for(unsigned i=1;i<=20;++i) {ASSERT_TRUE(h.manager->submit(h.batch(i)));h.run(10);}
  EXPECT_FALSE(h.manager->snapshot().fault);EXPECT_EQ(h.bus->counts[0xf5],120u);EXPECT_EQ(h.bus->counts[0x4b],0u);
}

TEST(BusManager, OldCommandAndWrongSourceCannotExecute) {
  Harness h; h.ready(); ASSERT_TRUE(h.manager->activate(Source::trajectory, h.now));
  auto batch = h.batch(1); batch.source = Source::jog;
  EXPECT_FALSE(h.manager->submit(batch));
  batch.source = Source::trajectory; batch.created -= 1s; batch.expires -= 1s;
  EXPECT_TRUE(h.manager->submit(batch)); h.run(2);
  EXPECT_FALSE(h.manager->snapshot().fault); EXPECT_EQ(h.bus->counts[0xf5], 0u);h.run(70);EXPECT_TRUE(h.manager->snapshot().fault);
}
TEST(BusManager, FeedbackAndWatchdogAreIndependentOfDiagnosticTraffic) {
  Harness h; h.ready(); ASSERT_TRUE(h.manager->activate(Source::trajectory, h.now));
  h.run(70);
  EXPECT_TRUE(h.manager->snapshot().fault); EXPECT_EQ(h.bus->counts[0xf7], 6u);
}
TEST(BusManager, LostPositionStopsEvenWhenEnableAndOtherRepliesContinue) {
  Harness h; h.ready(); ASSERT_TRUE(h.manager->activate(Source::trajectory, h.now));
  h.bus->missing_position = 4;
  for (std::uint64_t i = 1; i <= 8; ++i) { h.manager->submit(h.batch(i, 0)); h.run(20); }
  EXPECT_TRUE(h.manager->snapshot().fault);
  EXPECT_EQ(h.bus->counts[0xf7], 6u);
}
TEST(BusManager, BusOffAndUsbLossLatchFault) {
  for (const bool bus_off : {false, true}) {
    Harness h; h.ready(); ASSERT_TRUE(h.manager->activate(Source::trajectory, h.now));
    if (bus_off) h.bus->responses.push_front({0, {}, true, CAN_ERR_BUSOFF, h.now});
    else h.bus->disconnected = true;
    h.run(1); EXPECT_TRUE(h.manager->snapshot().fault);
    EXPECT_FALSE(h.manager->snapshot().active);
  }
}
TEST(BusManager, BoundsAndCommissioningGatesRejectMotion) {
  auto config = configured(); config.motion_enabled = false;
  Harness h(config); h.ready(); EXPECT_FALSE(h.manager->activate(Source::trajectory, h.now));
  auto broken = configured(); broken.axes[2].address = broken.axes[1].address;
  EXPECT_THROW(CanBusManager(broken, std::make_unique<Emulator>()), std::invalid_argument);
  EXPECT_THROW(absolute_position_request(1, 8388608, 10, 2), std::invalid_argument);
}
TEST(BusManager, SixAxesLongRunKeepsFeedbackAndBoundedState) {
  Harness h; h.ready(); ASSERT_TRUE(h.manager->activate(Source::trajectory, h.now));
  const auto rss = [] { std::size_t size{}, resident{}; std::ifstream("/proc/self/statm") >> size >> resident; return resident * static_cast<std::size_t>(sysconf(_SC_PAGESIZE)); };
  const auto memory_before = rss();
  const auto position_before = h.bus->counts[0x31];
  for (std::uint64_t i = 1; i <= 3000; ++i) { ASSERT_TRUE(h.manager->submit(h.batch(i, 0))); h.run(20); }
  const auto snapshot = h.manager->snapshot();
  EXPECT_FALSE(snapshot.fault) << snapshot.message;
  EXPECT_GE(snapshot.statistics.committed, 2999u);
  EXPECT_GE(h.bus->counts[0x31] - position_before, 6 * 1450u);
  EXPECT_LE(snapshot.statistics.scheduled_utilization, 0.5);
  EXPECT_LT(h.bus->responses.size(), 10u);
  EXPECT_LT(sizeof(CanBusManager), 2u * 1024u * 1024u);
  EXPECT_LE(rss(), memory_before + 4u * 1024u * 1024u);
  for (const auto &axis : configured().axes)
    EXPECT_TRUE(snapshot.devices[axis.address].position.fresh(h.now, 60ms));
}
TEST(BusManager, InvalidFramesCannotConsumePendingResponse) {
  Harness h; h.run(1);
  h.bus->responses.push_front({1, {0x40, 0x12}, false, 0, h.now});
  h.run(3);
  EXPECT_TRUE(h.manager->snapshot().devices[1].detected);
  EXPECT_GT(h.manager->snapshot().statistics.invalid, 0u);
}
TEST(BusManager, PositionCodecSupportsNegativeInt48AndConversion) {
  std::vector<std::uint8_t> frame{0x31, 0xff, 0xff, 0xff, 0xff, 0xc0, 0};
  frame.push_back(checksum(4, frame));
  EXPECT_EQ(parse_position(4, frame), -16384);
  auto axis = configured().axes[0]; axis.gear_ratio = 2; axis.direction = -1;
  EXPECT_NEAR(CanBusManager::position_to_joint(axis, -16384), 3.141592653589793, 1e-10);
  EXPECT_EQ(CanBusManager::joint_to_position(axis, 3.141592653589793), -16384);
}
TEST(BusManager, SourceLeaseLossStopsEvenIfControllerKeepsWriting) {
  auto config = configured(); config.source_lease_required = true;
  Harness h(config); h.ready();
  EXPECT_FALSE(h.manager->activate(Source::trajectory, h.now));
  h.manager->renew_source_lease(Source::jog, h.now);
  ASSERT_TRUE(h.manager->activate(Source::trajectory, h.now));
  for (std::uint64_t i = 1; i <= 15; ++i) {
    auto command = h.batch(i, 0); command.source = Source::jog;
    h.manager->submit(command); h.run(20);
  }
  EXPECT_TRUE(h.manager->snapshot().fault);
  EXPECT_EQ(h.bus->counts[0xf7], 6u);
}
TEST(BusManager, RejectsCycleWhenCriticalTrafficCannotFitBudget) {
  auto config = configured(); config.utilization_limit=.2;
  EXPECT_THROW(CanBusManager(config, std::make_unique<Emulator>()), std::invalid_argument);
}
TEST(BusManager, DelayedPositionDoesNotRefreshAnUnrelatedFieldOrSuppressTimeout) {
  Harness h; h.ready(); h.bus->missing_position = 1;
  h.run(100);
  const auto previous = h.manager->snapshot().devices[1].position;
  std::vector<std::uint8_t> reply{0x31, 0, 0, 0, 0, 0, 0}; reply.push_back(checksum(1, reply));
  h.bus->responses.push_back({1, reply, false, 0, h.now - 100ms});
  h.run(2);
  const auto next = h.manager->snapshot().devices[1].position;
  EXPECT_EQ(next.stamp, previous.stamp);
  EXPECT_GE(h.manager->snapshot().statistics.late, 1u);
}
TEST(BusManager, OversuppliedCommandsDoNotStarveFeedbackOrGrowQueue) {
  Harness h; h.ready(); ASSERT_TRUE(h.manager->activate(Source::trajectory, h.now));
  for (std::uint64_t sequence = 1; sequence <= 1000; ++sequence) {
    ASSERT_TRUE(h.manager->submit(h.batch(sequence, 0))); h.run(1);
  }
  const auto snapshot = h.manager->snapshot();
  EXPECT_FALSE(snapshot.fault) << snapshot.message;
  EXPECT_GT(snapshot.statistics.dropped_commands, 898u);
  EXPECT_LE(snapshot.statistics.scheduled_utilization, 0.5);
  EXPECT_LE(snapshot.statistics.committed, 101u);
  for (const auto &axis : configured().axes)
    EXPECT_TRUE(snapshot.devices[axis.address].position.fresh(h.now, 60ms));
}
TEST(BusManager, OfflineProbeIsLimitedAndReconnectRestoresNormalFeedback) {
  Harness h; h.ready(); h.bus->trace.clear(); h.bus->absent_address = 4; h.run(150);
  ASSERT_GE(h.manager->snapshot().devices[4].consecutive_timeouts, 3u);
  const auto requests = h.bus->position_requests[4];
  h.run(2000);
  EXPECT_LE(h.bus->position_requests[4] - requests, 3u);
  h.bus->absent_address = 0; h.run(1200);
  const auto device = h.manager->snapshot().devices[4];
  EXPECT_LT(device.consecutive_timeouts, 3u);
  EXPECT_TRUE(device.position.fresh(h.now, 60ms));
  EXPECT_TRUE(device.enable.fresh(h.now, 300ms));
  if(!device.mode.valid) {
    std::cerr << "Recovery trace CAN 4:";
    for(const auto &[address,opcode]:h.bus->trace) if(address==4) std::cerr << " " << unsigned(opcode);
    std::cerr << "; timeouts=" << device.timeouts << "; invalid=" << device.invalid << "; consecutive=" << device.consecutive_timeouts << "\n";
  }
  EXPECT_TRUE(device.mode.valid);
}

TEST(BusManager, ResetPreservesKnownDevicesButInvalidatesMeasurementsWithoutScanning) {
  Harness h;h.ready();const auto queries=h.bus->counts[0x40];
  h.manager->close();h.manager->reset_session(ResetMode::preserve_discovery);
  auto state=h.manager->snapshot();ASSERT_TRUE(state.devices[1].detected);
  EXPECT_FALSE(state.devices[1].position.valid);EXPECT_FALSE(state.devices[1].enable.valid);
  EXPECT_EQ(state.devices[1].last_ok,Time{});EXPECT_FALSE(state.devices[1].mode.valid);
  h.manager->open(h.now);h.run(200);
  state=h.manager->snapshot();EXPECT_FALSE(state.scanning);EXPECT_TRUE(state.devices[1].position.fresh(h.now,60ms));
  EXPECT_EQ(h.bus->counts[0x40],queries);EXPECT_EQ(h.bus->counts[0xf5],0u);EXPECT_EQ(h.bus->counts[0x4b],0u);
  h.manager->close();h.manager->reset_session();h.manager->open(h.now);h.run(200);
  EXPECT_FALSE(h.manager->snapshot().devices[1].detected);EXPECT_EQ(h.bus->counts[0x40],queries);
}

TEST(BusManager, ScanQueriesExactlyTwentyAddressesAndExcludesBroadcast) {
  Harness h;h.ready();std::vector<unsigned> addresses;
  for(const auto &[address,opcode]:h.bus->trace) if(opcode==0x40) addresses.push_back(address);
  ASSERT_EQ(addresses.size(),20u);
  for(unsigned i=0;i<20;++i) EXPECT_EQ(addresses[i],i+1);
}
