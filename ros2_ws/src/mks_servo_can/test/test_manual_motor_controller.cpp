#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <linux/can/error.h>
#include "mks_servo_can/manual_motor_controller.hpp"
using namespace mks_servo_can;
using namespace std::chrono_literals;

namespace {
struct Motor {
  bool enabled{}, stalled{};
  unsigned mode{2}, microstep{16}, sync{1}, heartbeat{};
  double rpm{}, target{}, ticks{};
};
class Port final : public Transport {
 public:
  bool opened{}, disconnected{}, echo{};
  unsigned drop_command{}, no_read_command{}, corrupt_command{}, refuse_command{};
  std::chrono::milliseconds zero_delay{1};
  int idle_speed{};
  bool receive_clock_skew{};
  Time now{1s};
  std::map<unsigned, Motor> motors{{1, {}}, {2, {}}, {4, {}}};
  std::deque<Frame> rx;
  std::array<unsigned, 256> counts{};
  std::vector<std::pair<unsigned, std::vector<std::uint8_t>>> writes;
  void open(const std::string &) override { opened = true; }
  void close() override { opened = false; }
  bool is_open() const override { return opened; }
  void advance() {
    for (auto &[id, m] : motors) {
      (void)id;
      if (!m.enabled) { m.rpm = 0; continue; }
      const auto acceleration = speed_scale(m.microstep) / .01275;
      m.rpm += std::clamp(m.target - m.rpm, -acceleration * .001, acceleration * .001);
      m.ticks += m.rpm * 16384 / 60 * .001;
    }
  }
  void send(std::uint16_t address, const std::vector<std::uint8_t> &data) override {
    if (disconnected) throw std::runtime_error("USB odłączone");
    ++counts[data[0]];
    if (data[0] == 0xf6 || data[0] == 0xf3 || data[0] == 0x92) writes.emplace_back(address, data);
    if (echo) rx.push_back({address, data, false, 0, now + 1ms});
    if (!motors.count(address)) return;
    auto &m = motors[address];
    const auto cmd = data[0];
    if (drop_command && cmd == drop_command) return;
    std::vector<std::uint8_t> reply{cmd};
    switch (cmd) {
      case 0x40: reply = {0x40, 0x12, 1, 0, 9, 0x20}; break;
      case 0: {
        const auto parameter = data[1]; if (parameter == no_read_command) return;
        reply = {parameter};
        if (parameter == 0x98) reply.insert(reply.end(), {0, 0, static_cast<std::uint8_t>(m.heartbeat >> 8), static_cast<std::uint8_t>(m.heartbeat)});
        else reply.push_back(parameter == 0x82 ? m.mode : parameter == 0x84 ? m.microstep : m.sync);
        break;
      }
      case 0x31: {
        const auto ticks = static_cast<std::uint64_t>(static_cast<std::int64_t>(std::llround(m.ticks)));
        for (int shift = 40; shift >= 0; shift -= 8) reply.push_back(ticks >> shift);
        break;
      }
      case 0x32: {
        const auto speed = static_cast<std::uint16_t>(static_cast<std::int16_t>(m.rpm == 0 ? idle_speed : std::lround(-m.rpm)));
        reply.insert(reply.end(), {static_cast<std::uint8_t>(speed >> 8), static_cast<std::uint8_t>(speed)}); break;
      }
      case 0x3a: reply.push_back(m.enabled); break;
      case 0x3e: reply.push_back(m.stalled); break;
      case 0xf1: reply.push_back(m.rpm == 0 && m.target == 0 ? 1 : m.target == m.rpm ? 4 : m.target == 0 ? 3 : 2); break;
      case 0xf3: if (cmd != refuse_command) m.enabled = data[1]; reply.push_back(cmd == refuse_command ? 0 : 1); break;
      case 0x82: m.mode = data[1]; reply.push_back(1); break;
      case 0x4a: m.sync = data[1]; reply.push_back(1); break;
      case 0x98: m.heartbeat = (unsigned(data[1]) << 24) | (unsigned(data[2]) << 16) | (unsigned(data[3]) << 8) | data[4]; reply.push_back(1); break;
      case 0x92: m.ticks = 0; reply.push_back(1); break;
      case 0xf7: m.target = m.rpm = 0; reply.push_back(1); break;
      case 0xf6: {
        const auto raw = (unsigned(data[1] & 0xf) << 8) | data[2];
        if (m.enabled) m.target = raw * speed_scale(m.microstep) * (data[1] & 0x80 ? 1 : -1);
        reply.push_back(m.enabled ? 1 : 0); break;
      }
      default: reply.push_back(1); break;
    }
    reply.push_back(checksum(address, reply));
    if (cmd == corrupt_command && cmd) ++reply.back();
    rx.push_back({address, reply, false, 0, now + (cmd == 0x92 ? zero_delay : 1ms) +
        (receive_clock_skew ? std::chrono::microseconds(10) : std::chrono::microseconds(0))});
  }
  std::optional<Frame> receive() override {
    if (disconnected) throw std::runtime_error("USB odłączone");
    if (rx.empty() || rx.front().received_at > now + (receive_clock_skew ? std::chrono::microseconds(10) : std::chrono::microseconds(0))) return {};
    const auto result = rx.front(); rx.pop_front(); return result;
  }
};
struct Rig {
  Port *port;
  CanBusManager bus;
  Time now{1s};
  static std::unique_ptr<Port> make(Port *&port) { auto p = std::make_unique<Port>(); port = p.get(); return p; }
  Rig() : bus(BusConfig{}, make(port)) { bus.open(now); bus.start_scan(); step(180); }
  void step(unsigned count = 1) {
    for (unsigned i = 0; i < count; ++i) { now += 1ms; port->now = now; port->advance(); bus.tick(now); }
  }
  ManualMotorController &manual() { return bus.manual(); }
  ManualState state() { return manual().snapshot(now); }
  void enable(unsigned axis = 1) {
    ASSERT_TRUE(manual().set_enable(axis, true, now)); step(160);
    ASSERT_FALSE(bus.snapshot().fault) << state().message;
    ASSERT_TRUE(state().axes[axis - 1].can_jog) << state().message;
  }
  bool start(unsigned axis = 1, const std::string &owner = "panel-a", unsigned seq = 1) {
    return manual().jog(JogAction::start, axis, 1, 30, owner, seq, now, now);
  }
  void held(unsigned duration, unsigned &seq, unsigned axis = 1, const std::string &owner = "panel-a") {
    for (unsigned i = 0; i < duration / 50; ++i) {
      step(50); ASSERT_TRUE(manual().jog(JogAction::keepalive, axis, 0, 0, owner, ++seq, now, now));
    }
  }
};
}
TEST(ManualProtocol, DirectionSpeedAndSignedFeedback) {
  EXPECT_EQ(speed_request(1, true, 320, 2), (std::vector<std::uint8_t>{0xf6, 0x81, 0x40, 2, 0xba}));
  EXPECT_EQ(enable_request(1, false), request(1, 0xf3, {0}));
  EXPECT_EQ(zero_request(1), request(1, 0x92));
  auto answer = request(4, 0x32, {0xff, 0xe2}); EXPECT_EQ(parse_speed(4, answer), -30);
  EXPECT_FALSE(parse_speed(1, answer)); answer.back()++; EXPECT_FALSE(parse_speed(4, answer));
  EXPECT_FALSE(parse_speed(4, {0x32, 0})); EXPECT_THROW(speed_request(1, false, 3001), std::invalid_argument);
  EXPECT_THROW(speed_request(1, false, 1, 0), std::invalid_argument);
  EXPECT_DOUBLE_EQ(speed_scale(128), .125); EXPECT_THROW(speed_scale(0), std::invalid_argument);
}
TEST(ManualMotor, OnlyKnownAxesAndNoAutomaticWrites) {
  Rig r; EXPECT_TRUE(r.bus.snapshot().devices[1].detected); EXPECT_TRUE(r.bus.snapshot().devices[2].detected);
  EXPECT_TRUE(r.bus.snapshot().devices[4].detected); EXPECT_FALSE(r.bus.snapshot().devices[3].detected);
  EXPECT_TRUE(r.port->writes.empty()); EXPECT_FALSE(r.start()); EXPECT_FALSE(r.manual().set_enable(3, true, r.now));
  r.enable(4); EXPECT_EQ(r.port->motors[4].mode, 5u); EXPECT_EQ(r.port->motors[4].sync, 0u);
  EXPECT_EQ(r.port->motors[4].heartbeat, 300u); EXPECT_FALSE(r.bus.snapshot().scanning);
  EXPECT_FALSE(r.port->motors[1].enabled); EXPECT_FALSE(r.port->motors[2].enabled);
}
TEST(ManualMotor, PreparationStopsWithEnableOffWithoutUsingSpeedCommand) {
  Rig r; r.port->motors[1].target = 30; r.enable();
  EXPECT_EQ(r.port->counts[0xf6], 0u);
  EXPECT_EQ(r.port->counts[0xf7], 1u);
  EXPECT_EQ(r.port->motors[1].target, 0);
  EXPECT_TRUE(r.port->motors[1].enabled);
}
TEST(ManualMotor, OneOwnerReleaseAndDisable) {
  Rig r; r.enable(1); r.enable(2); ASSERT_TRUE(r.start()); unsigned seq = 1;
  EXPECT_FALSE(r.start(2, "panel-b")); EXPECT_FALSE(r.bus.start_scan()); EXPECT_FALSE(r.bus.service_allowed());
  r.held(500, seq); EXPECT_EQ(r.state().axes[0].motion, "w prędkości");
  EXPECT_GT(r.state().axes[0].speed.value, 20);
  EXPECT_FALSE(r.manual().jog(JogAction::keepalive, 1, 0, 0, "panel-b", 2, r.now, r.now));
  ASSERT_TRUE(r.manual().jog(JogAction::stop, 1, 0, 0, "panel-a", ++seq, r.now, r.now));
  r.step(80); EXPECT_EQ(r.state().axes[0].motion, "hamuje"); EXPECT_FALSE(r.start(2, "panel-b"));
  r.step(550); EXPECT_FALSE(r.state().busy); EXPECT_EQ(r.state().axes[0].motion, "zatrzymany");
  EXPECT_FALSE(r.manual().jog(JogAction::keepalive, 1, 0, 0, "panel-a", ++seq, r.now, r.now));
  ASSERT_TRUE(r.manual().set_enable(1, false, r.now)); r.step(100);
  EXPECT_FALSE(r.port->motors[1].enabled); EXPECT_FALSE(r.state().axes[0].can_jog);
  EXPECT_TRUE(r.start(2, "panel-b"));
}
TEST(ManualMotor, LostLeaseStopsWithoutResuming) {
  Rig r; r.enable(); ASSERT_TRUE(r.start()); r.step(850);
  EXPECT_EQ(r.port->motors[1].target, 0); EXPECT_FALSE(r.state().busy); EXPECT_FALSE(r.state().fault);
  EXPECT_FALSE(r.manual().jog(JogAction::keepalive, 1, 0, 0, "panel-a", 2, r.now, r.now));
}
TEST(ManualMotor, ReleaseUnlocksOnEncoderConfirmationWithoutExtraSpeedCycles) {
  Rig r; r.enable(1); r.enable(2); ASSERT_TRUE(r.start()); unsigned seq = 1;
  r.held(500, seq);
  ASSERT_TRUE(r.manual().jog(JogAction::stop, 1, 0, 0, "panel-a", ++seq, r.now, r.now));
  unsigned elapsed = 0;
  while (r.state().busy && elapsed < 600) {
    if (r.port->motors[1].rpm != 0) EXPECT_FALSE(r.start(2, "panel-b"));
    r.step(); ++elapsed;
  }
  EXPECT_GE(elapsed, 383u);  // Rampa 30 RPM / 78,43 RPM/s musi się skończyć.
  EXPECT_LT(elapsed, 500u);
  EXPECT_EQ(r.port->motors[1].rpm, 0);
  EXPECT_TRUE(r.state().axes[0].can_jog);
  r.step();  // Nowe naciśnięcie musi być nowsze od bariery STOP.
  EXPECT_TRUE(r.start(2, "panel-b"));
}
TEST(ManualMotor, DisplayGraceDoesNotRelaxPositionDeadlineForMotion) {
  Rig r; r.enable(); ASSERT_TRUE(r.start()); unsigned seq = 1; r.held(100, seq);
  r.port->drop_command = 0x31; r.step(85);
  EXPECT_EQ(r.state().operation, "stopping");
  EXPECT_TRUE(r.state().axes[0].position.valid);
  EXPECT_TRUE(r.state().axes[0].available);
  EXPECT_FALSE(r.state().axes[0].can_jog);
}
TEST(ManualMotor, EncoderOscillationWithinOneDegreeKeepsJogReadyAfterStop) {
  Rig r; r.enable(); ASSERT_TRUE(r.start()); unsigned seq = 1;
  r.held(500, seq);
  ASSERT_TRUE(r.manual().jog(JogAction::stop, 1, 0, 0, "panel-a", ++seq, r.now, r.now));
  r.step(600); ASSERT_FALSE(r.state().busy);
  const auto center = std::llround(r.port->motors[1].ticks);
  for (unsigned i = 0; i < 250; ++i) {
    // ±45 impulsów to ±0,989°, a pełny rozrzut to prawie 2°.
    r.port->motors[1].ticks = center + (i % 2 ? 45 : -45);
    for (unsigned j = 0; j < 20; ++j) {
      r.step(); const auto axis = r.state().axes[0];
      ASSERT_TRUE(axis.can_jog) << "Próbka " << i;
      EXPECT_TRUE(axis.enable.valid); EXPECT_TRUE(axis.enable.value);
      EXPECT_EQ(axis.motion, "zatrzymany"); EXPECT_DOUBLE_EQ(axis.speed.value, 0);
    }
  }
  EXPECT_FALSE(r.state().axes[0].can_zero);  // Dokładność zerowania nie została rozluźniona.
  EXPECT_FALSE(r.manual().zero(1, r.now));
  EXPECT_TRUE(r.start());
}
TEST(ManualMotor, JogRestToleranceIsAnchoredNotAppliedToEachPositionDelta) {
  Rig r; r.enable();
  for (unsigned i = 1; i <= 20; ++i) {
    r.port->motors[1].ticks = 20 * i; r.step(20);
    if (i >= 3) EXPECT_FALSE(r.state().axes[0].can_jog);
  }
  EXPECT_FALSE(r.start());
  r.step(100); EXPECT_TRUE(r.state().axes[0].can_jog);
}
TEST(ManualMotor, JogToleranceDoesNotHideBrakingOrStaleFeedback) {
  Rig r; r.enable();
  r.port->motors[1].target = 1; r.step(100);
  EXPECT_FALSE(r.state().axes[0].can_jog); EXPECT_FALSE(r.start());
  r.port->motors[1].target = 0; r.step(200);
  ASSERT_TRUE(r.state().axes[0].can_jog);
  r.port->drop_command = 0xf1; r.step(160);
  EXPECT_FALSE(r.state().axes[0].can_jog); EXPECT_FALSE(r.start());
  EXPECT_EQ(r.state().axes[0].motion, "stan nieznany");
  r.port->drop_command = 0; r.step(200);
  ASSERT_TRUE(r.state().axes[0].can_jog);
  r.port->motors[1].stalled = true; r.step(150);
  EXPECT_FALSE(r.state().axes[0].can_jog); EXPECT_FALSE(r.start());
}
TEST(ManualMotor, DisableDuringJogWaitsForRest) {
  Rig r; r.enable(); ASSERT_TRUE(r.start()); unsigned seq = 1; r.held(500, seq);
  ASSERT_TRUE(r.manual().set_enable(1, false, r.now)); r.step(100); EXPECT_TRUE(r.port->motors[1].enabled);
  r.step(600); EXPECT_FALSE(r.port->motors[1].enabled); EXPECT_FALSE(r.state().busy);
}
TEST(ManualMotor, ZeroOnlyStationaryAndConfirmed) {
  Rig r; r.port->motors[1].ticks = 1000; r.step(30); EXPECT_FALSE(r.manual().zero(1, r.now));
  r.enable(); ASSERT_TRUE(r.manual().zero(1, r.now)); r.step(160);
  EXPECT_NEAR(r.state().axes[0].position.value, 0, .01); EXPECT_FALSE(r.state().busy);
  ASSERT_TRUE(r.start()); r.step(20); EXPECT_FALSE(r.manual().zero(1, r.now));
}
TEST(ManualMotor, ConfigurationAckMayTakeLongerThanReadTimeout) {
  Rig r; r.enable(); r.port->zero_delay = 35ms;
  ASSERT_TRUE(r.manual().zero(1, r.now)); r.step(160);
  EXPECT_FALSE(r.state().fault); EXPECT_FALSE(r.state().busy);
  EXPECT_EQ(r.state().message, "Pozycja zero potwierdzona");
}
TEST(ManualMotor, CachedReportedSpeedRequiresStablePositionAndStoppedStatus) {
  Rig r; r.port->idle_speed = 3; r.enable();
  EXPECT_TRUE(r.state().axes[0].can_jog);
  ASSERT_TRUE(r.manual().zero(1, r.now)); r.step(180);
  EXPECT_FALSE(r.state().fault); EXPECT_TRUE(r.state().axes[0].can_jog);
  r.port->motors[1].target = 1; r.step(180);
  EXPECT_FALSE(r.state().axes[0].can_jog);
  EXPECT_NE(r.state().axes[0].motion, "zatrzymany");
}
TEST(ManualMotor, ReceiveTimestampAfterTickEntryDoesNotInvalidateFreshSamples) {
  Rig r; r.port->receive_clock_skew = true;
  ASSERT_TRUE(r.manual().set_enable(1, true, r.now)); r.step(500);
  const auto state = r.manual().snapshot(r.now + 1ms);
  EXPECT_FALSE(state.fault); EXPECT_FALSE(state.busy); EXPECT_TRUE(state.axes[0].can_jog);
}
TEST(ManualMotor, SoftLimitIncludesBrakingDistance) {
  Rig r; r.enable(); r.port->motors[1].ticks = 3590 * 16384 / 360.0; r.step(160);
  ASSERT_TRUE(r.state().axes[0].can_jog); EXPECT_FALSE(r.start());
  r.port->motors[1].ticks = 3500 * 16384 / 360.0; r.step(160);
  ASSERT_TRUE(r.start()); unsigned seq = 1;
  for (int n = 0; n < 30; ++n) {
    r.step(50);
    r.manual().jog(JogAction::keepalive, 1, 0, 0, "panel-a", ++seq, r.now, r.now);
  }
  EXPECT_FALSE(r.state().busy); EXPECT_LE(r.state().axes[0].position.value, 3600);
}
TEST(ManualMotor, RefusedEnableNeverConfirmsReady) {
  Rig r; r.port->refuse_command = 0xf3;
  ASSERT_TRUE(r.manual().set_enable(1, true, r.now)); r.step(100);
  EXPECT_TRUE(r.state().fault); EXPECT_FALSE(r.state().axes[0].can_jog);
  EXPECT_FALSE(r.port->motors[1].enabled); const auto tx = r.bus.snapshot().statistics.tx; r.step(1000);
  EXPECT_EQ(tx, r.bus.snapshot().statistics.tx); EXPECT_TRUE(r.bus.service_allowed());
}
TEST(ManualMotor, MissingStopResponseLatchesFaultAndSilencesBus) {
  Rig r; r.enable(); ASSERT_TRUE(r.start()); unsigned seq = 1; r.held(500, seq);
  r.port->drop_command = 0xf6;
  ASSERT_TRUE(r.manual().jog(JogAction::stop, 1, 0, 0, "panel-a", ++seq, r.now, r.now)); r.step(100);
  EXPECT_TRUE(r.state().fault); EXPECT_TRUE(r.state().busy); EXPECT_FALSE(r.start(2, "panel-b"));
  const auto tx = r.bus.snapshot().statistics.tx; r.step(500); EXPECT_EQ(tx, r.bus.snapshot().statistics.tx);
  r.bus.close(); r.bus.reset_session(); r.bus.open(r.now); EXPECT_FALSE(r.state().busy);
}
TEST(ManualMotor, OldSequenceAndTimestampRejected) {
  Rig r; r.enable(); EXPECT_FALSE(r.manual().jog(JogAction::start, 1, 1, 30, "panel-a", 1, r.now - 101ms, r.now));
  ASSERT_TRUE(r.start()); r.step(30);
  EXPECT_FALSE(r.manual().jog(JogAction::keepalive, 1, 0, 0, "panel-a", 1, r.now, r.now));
  EXPECT_FALSE(r.manual().jog(JogAction::keepalive, 1, 0, 0, "panel-a", 2, r.now + 1ms, r.now));
}
TEST(ManualMotor, BusOffAndUsbNeverResume) {
  for (bool usb : {false, true}) {
    Rig r; r.enable(); ASSERT_TRUE(r.start()); r.step(20);
    if (usb) r.port->disconnected = true;
    else r.port->rx.push_back({0, {}, true, CAN_ERR_BUSOFF, r.now});
    r.step(10); EXPECT_TRUE(r.state().fault); EXPECT_FALSE(r.state().axes[0].can_jog);
    EXPECT_EQ(r.state().axes[0].motion, "stan nieznany");
  }
}
TEST(ManualMotor, StopBeforeDelayedStartCreatesTimeBarrier) {
  Rig r; r.enable(); const auto issued = r.now; r.step(1);
  EXPECT_FALSE(r.manual().jog(JogAction::stop, 1, 0, 0, "panel-a", 2, r.now, r.now));
  EXPECT_FALSE(r.manual().jog(JogAction::start, 1, 1, 30, "panel-a", 1, issued, r.now));
  EXPECT_EQ(r.port->motors[1].target, 0);
}
TEST(ManualMotor, LeftArrowMovesNegativeAndStops) {
  Rig r; r.enable(); ASSERT_TRUE(r.manual().jog(JogAction::start, 1, -1, 30, "panel-a", 1, r.now, r.now));
  unsigned seq = 1; r.held(500, seq); EXPECT_LT(r.state().axes[0].position.value, 0);
  EXPECT_LT(r.state().axes[0].speed.value, 0); r.step(850); EXPECT_FALSE(r.state().busy);
}
TEST(ManualMotor, UnsupportedParameterReadDoesNotEnableMotor) {
  Rig r; r.port->no_read_command = 0x4a;
  ASSERT_TRUE(r.manual().set_enable(1, true, r.now)); r.step(100);
  EXPECT_TRUE(r.state().fault); EXPECT_FALSE(r.port->motors[1].enabled);
}
TEST(ManualMotor, FullScanEchoCannotInventDevices) {
  Rig r; r.port->echo = true; r.step(44000);
  const auto snapshot = r.bus.snapshot(); EXPECT_EQ(snapshot.checked, 2047u);
  unsigned count = 0; for (const auto &d : snapshot.devices) count += d.detected;
  EXPECT_EQ(count, 3u); EXPECT_TRUE(r.port->writes.empty());
}

namespace {
RobotProfile joint_profile() {
  RobotProfile profile; profile.revision = "profile-1"; profile.changed[0] = true;
  profile.axes[0] = {true, 10, -1, 0, -1, 1, .5, 10};
  return profile;
}
void apply_profile(Rig &r, const RobotProfile &profile, const std::string &operation = "apply-1", const std::string &previous = "initial") {
  const auto prepare = r.bus.configure(ConfigurationPhase::prepare, operation, previous, profile, r.now);
  ASSERT_TRUE(prepare.accepted) << prepare.message;
  ASSERT_TRUE(r.bus.configure(ConfigurationPhase::commit, operation, "", profile, r.now).accepted);
  ASSERT_TRUE(r.bus.configure(ConfigurationPhase::finish, operation, "", profile, r.now).accepted);
}
}
TEST(Configuration, StartupBlocksEnableUntilBootstrapAndNeverEnablesAutomatically) {
  Rig r; r.bus.require_configuration();
  EXPECT_FALSE(r.manual().set_enable(1, true, r.now)); EXPECT_TRUE(r.bus.service_allowed());
  RobotProfile profile;
  ASSERT_TRUE(r.bus.configure(ConfigurationPhase::bootstrap, "boot", "", profile, r.now).accepted);
  EXPECT_FALSE(r.manual().set_enable(1, true, r.now));
  ASSERT_TRUE(r.bus.configure(ConfigurationPhase::finish, "boot", "", profile, r.now).accepted);
  EXPECT_TRUE(r.port->writes.empty());r.enable(2);
}
TEST(Configuration, EnableStopAndRevisionGuardsAndRollback) {
  Rig r; auto profile = joint_profile(); r.enable();
  EXPECT_FALSE(r.bus.configure(ConfigurationPhase::prepare,"apply","initial",profile,r.now).accepted);
  ASSERT_TRUE(r.manual().set_enable(1,false,r.now));r.step(160);
  EXPECT_FALSE(r.bus.configure(ConfigurationPhase::prepare,"apply","wrong",profile,r.now).accepted);
  ASSERT_TRUE(r.bus.configure(ConfigurationPhase::prepare,"apply","initial",profile,r.now).accepted);
  EXPECT_FALSE(r.manual().set_enable(2,true,r.now));EXPECT_FALSE(r.bus.start_scan());
  ASSERT_TRUE(r.bus.configure(ConfigurationPhase::commit,"apply","",profile,r.now).accepted);
  EXPECT_TRUE(r.state().axes[0].configured);
  ASSERT_TRUE(r.bus.configure(ConfigurationPhase::rollback,"apply","",profile,r.now).accepted);
  EXPECT_FALSE(r.state().axes[0].configured);
  EXPECT_FALSE(r.bus.configure(ConfigurationPhase::commit,"apply","",profile,r.now).accepted);
  ASSERT_TRUE(r.bus.configure(ConfigurationPhase::finish,"apply","",profile,r.now).accepted);
  EXPECT_FALSE(r.state().busy);EXPECT_FALSE(r.port->motors[1].enabled);
}
TEST(Configuration, CancelledLatePrepareAndCommitCannotApply) {
  Rig r; auto profile=joint_profile();
  ASSERT_TRUE(r.bus.configure(ConfigurationPhase::rollback,"cancelled","",profile,r.now).accepted);
  ASSERT_TRUE(r.bus.configure(ConfigurationPhase::finish,"cancelled","",profile,r.now).accepted);
  EXPECT_FALSE(r.bus.configure(ConfigurationPhase::prepare,"cancelled","initial",profile,r.now).accepted);
  EXPECT_FALSE(r.bus.configure(ConfigurationPhase::commit,"cancelled","",profile,r.now).accepted);
  EXPECT_FALSE(r.state().axes[0].configured);
}
TEST(Configuration, JointJogUsesRatioDirectionAndFloorsSpeedWithoutLegacyBypass) {
  Rig r; apply_profile(r,joint_profile());r.enable();
  EXPECT_TRUE(r.state().axes[0].configured);EXPECT_TRUE(r.state().axes[0].calibration_valid);
  EXPECT_FALSE(r.start());
  EXPECT_FALSE(r.manual().jog_joint(JogAction::start,1,1,.1,"stale","owner",1,r.now,r.now));
  EXPECT_FALSE(r.manual().jog_joint(JogAction::start,1,1,.001,"profile-1","owner",1,r.now,r.now));
  ASSERT_TRUE(r.manual().jog_joint(JogAction::start,1,1,.1,"profile-1","owner",1,r.now,r.now));
  r.step(160);
  EXPECT_DOUBLE_EQ(r.port->motors[1].target,-9);
  EXPECT_GT(r.state().axes[0].joint_position.value,0);EXPECT_LT(r.state().axes[0].position.value,0);
  EXPECT_FALSE(r.manual().jog(JogAction::start,2,1,30,"other",1,r.now,r.now));
  ASSERT_TRUE(r.manual().jog_joint(JogAction::stop,1,0,0,"old-revision","owner",2,r.now,r.now));
  r.step(700);EXPECT_FALSE(r.state().busy);
}
TEST(Configuration, JointLimitIncludesBrakingAndWatchdogStillStops) {
  Rig r;auto profile=joint_profile();profile.axes[0].lower=-.03;profile.axes[0].upper=.03;
  apply_profile(r,profile);r.enable();
  ASSERT_TRUE(r.manual().jog_joint(JogAction::start,1,1,.1,"profile-1","owner",1,r.now,r.now));
  unsigned sequence=1;
  for(int i=0;i<30&&r.state().busy;i++){
    r.step(50);r.manual().jog_joint(JogAction::keepalive,1,0,0,"profile-1","owner",++sequence,r.now,r.now);
  }
  EXPECT_FALSE(r.state().busy);EXPECT_LE(r.state().axes[0].joint_position.value,.03);
  r.step(100);
  ASSERT_TRUE(r.manual().jog_joint(JogAction::start,1,-1,.1,"profile-1","owner",++sequence,r.now,r.now));
  r.step(900);EXPECT_FALSE(r.state().busy);EXPECT_FALSE(r.state().fault);
}
TEST(Configuration, UnsupportedAccelerationRejectsPreparationWithoutEnablingOrFaultingOtherAxes) {
  Rig r;auto profile=joint_profile();profile.axes[0].acceleration=.001;apply_profile(r,profile);
  ASSERT_TRUE(r.manual().set_enable(1,true,r.now));r.step(300);
  EXPECT_FALSE(r.port->motors[1].enabled);EXPECT_FALSE(r.state().axes[0].can_jog);
  EXPECT_NE(r.state().axes[0].configuration_message.find("przyspieszenie"),std::string::npos);
  EXPECT_FALSE(r.state().fault);r.enable(2);
}
TEST(Configuration, AccelerationLimitUsesJointDegreesAndReportsBothValues) {
  Rig r; auto profile = joint_profile();
  profile.axes[0].gear_ratio = 1;
  profile.axes[0].acceleration = 100 * std::acos(-1.0) / 180;
  apply_profile(r, profile);
  ASSERT_TRUE(r.manual().set_enable(1, true, r.now)); r.step(300);
  EXPECT_FALSE(r.port->motors[1].enabled);
  EXPECT_FALSE(r.state().axes[0].can_jog);
  EXPECT_FALSE(r.state().fault);
  const auto message = r.state().axes[0].configuration_message;
  EXPECT_NE(message.find("470.6 °/s²"), std::string::npos) << message;
  EXPECT_NE(message.find("100.0 °/s²"), std::string::npos) << message;
  EXPECT_EQ(message.find("rad/s"), std::string::npos) << message;
}
TEST(Configuration, AccelerationLimitAccountsForActualGearRatio) {
  Rig r; auto profile = joint_profile();
  profile.axes[0].gear_ratio = 5;
  profile.axes[0].acceleration = 100 * std::acos(-1.0) / 180;
  apply_profile(r, profile); r.enable();
  EXPECT_TRUE(r.port->motors[1].enabled);
  EXPECT_TRUE(r.state().axes[0].can_jog);
}
TEST(Configuration, RepresentableAccelerationLimitAllowsPreparation) {
  Rig r; auto profile = joint_profile();
  profile.axes[0].gear_ratio = 1;
  profile.axes[0].acceleration = 500 * std::acos(-1.0) / 180;
  apply_profile(r, profile); r.enable();
  EXPECT_TRUE(r.port->motors[1].enabled);
  EXPECT_TRUE(r.state().axes[0].can_jog);
}
TEST(Configuration, ZeroInvalidatesCalibrationAndResetCannotBypassIt) {
  Rig r;apply_profile(r,joint_profile());r.enable();r.step(200);
  ASSERT_TRUE(r.manual().zero(1,r.now));r.step(180);
  EXPECT_FALSE(r.state().axes[0].calibration_valid);EXPECT_FALSE(r.state().axes[0].can_jog);
  r.bus.close();r.bus.reset_session();r.bus.open(r.now);
  EXPECT_FALSE(r.state().axes[0].calibration_valid);
}
TEST(Configuration, BootstrapRestoresPersistedInvalidCalibration) {
  Rig r;r.bus.require_configuration();auto profile=joint_profile();profile.calibration_invalid[0]=true;
  ASSERT_TRUE(r.bus.configure(ConfigurationPhase::bootstrap,"boot","",profile,r.now).accepted);
  ASSERT_TRUE(r.bus.configure(ConfigurationPhase::finish,"boot","",profile,r.now).accepted);
  EXPECT_TRUE(r.state().axes[0].configured);EXPECT_FALSE(r.state().axes[0].calibration_valid);
}
TEST(Configuration, RejectedApplyAndCancellationDoNotInterruptExistingJog) {
  Rig r;r.enable();ASSERT_TRUE(r.start());auto profile=joint_profile();
  EXPECT_FALSE(r.bus.configure(ConfigurationPhase::prepare,"rejected","initial",profile,r.now).accepted);
  ASSERT_TRUE(r.bus.configure(ConfigurationPhase::rollback,"rejected","",profile,r.now).accepted);
  ASSERT_TRUE(r.bus.configure(ConfigurationPhase::finish,"rejected","",profile,r.now).accepted);
  r.step(40);EXPECT_EQ(r.state().operation,"jog");EXPECT_GT(r.port->motors[1].target,0);
}
TEST(Configuration, BusOffBetweenPrepareAndCommitCannotCommit) {
  Rig r;auto profile=joint_profile();ASSERT_TRUE(r.bus.configure(ConfigurationPhase::prepare,"apply","initial",profile,r.now).accepted);
  r.port->rx.push_back({0,{},true,CAN_ERR_BUSOFF,r.now});r.step(5);
  EXPECT_FALSE(r.bus.configure(ConfigurationPhase::commit,"apply","",profile,r.now).accepted);
  EXPECT_FALSE(r.state().axes[0].configured);
  ASSERT_TRUE(r.bus.configure(ConfigurationPhase::rollback,"apply","",profile,r.now).accepted);
  ASSERT_TRUE(r.bus.configure(ConfigurationPhase::finish,"apply","",profile,r.now).accepted);
  EXPECT_TRUE(r.state().fault);EXPECT_TRUE(r.bus.service_allowed());
}
