#include <gtest/gtest.h>
#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include <algorithm>
#include <deque>
#include <map>
#include <linux/can/error.h>
using namespace mks_servo_can;
using namespace std::chrono_literals;
namespace {
class Emulator final : public Transport {
 public:
  bool opened{false}, disconnected{false}, own_echo{false}, complete_before_ack{false};
  std::uint16_t missing_ack{}, missing_position{};
  std::uint16_t absent_address{};
  std::array<bool,7> enabled{false,true,true,true,true,true,true}, sync{};
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
        if(data[1]==0x98) reply={0x98,0,0,1,44};
        else reply={data[1],static_cast<uint8_t>(data[1]==0x82 ? 5 : data[1]==0x84 ? 16 : sync[address])};
        break;
      case 0x4a: sync[address]=data[1];reply={0x4a,1};break;
      case 0x82: case 0x8c: case 0x98: case 0xf7: reply={data[0],1};break;
      case 0xf3: enabled[address]=data[1];reply={0xf3,1};break;
      case 0x32: reply={0x32,0,0};break;
      case 0x92: positions[address]=0;reply={0x92,1};break;
      case 0x3a: reply = {0x3a, static_cast<uint8_t>(enabled[address])}; break;
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
        if(complete_before_ack) {
          auto complete=std::vector<uint8_t>{0xf5,2};complete.push_back(checksum(address,complete));
          responses.push_back({address,complete,false,0,now+1ms});
        }
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
    config.axes[i] = {static_cast<std::uint16_t>(i + 1), 10, 1, 0, -2, 2, 1, 100, 0.2, 1};
  return config;
}
struct Harness {
  Emulator *bus;
  std::unique_ptr<CanBusManager> manager;
  Time now{1s};
  explicit Harness(BusConfig config = configured()) {
    auto transport = std::make_unique<Emulator>(); bus = transport.get(); bus->now = now;
    manager = std::make_unique<CanBusManager>(config, std::move(transport));
    manager->open(now);
    manager->require_configuration();
    RobotProfile profile;profile.revision="bench";profile.changed.fill(true);
    for(auto &a:profile.axes) a={true,10,1,0,-2,2,1,100};
    if(!manager->configure(ConfigurationPhase::bootstrap,"boot","",profile,now).accepted ||
       !manager->configure(ConfigurationPhase::finish,"boot","",profile,now).accepted) throw std::runtime_error("bootstrap");
    config.program_enabled=true;config.program_revision="bench";
    config.program_stop_verified=config.program_sync_verified=config.program_staged_reset_verified=true;
    manager->configure_program_authorization(config);
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

namespace {
void enable_axes(Harness &h) {
  for(unsigned j=1;j<=6;++j) if(j!=h.bus->absent_address) {
    ASSERT_TRUE(h.manager->manual().set_enable(j,true,h.now));h.run(200);
    ASSERT_TRUE(h.manager->manual().prepared(j)) << h.manager->manual().snapshot(h.now).message;
  }
}
PreparedMotionProgram hold(Harness &h,bool partial=false) {
  h.manager->update_motion_model("model");
  h.manager->update_motion_settings("settings",partial,h.now);
  PreparedMotionProgram p;p.id="plan";p.model_id="model";p.revision="bench";p.settings_revision="settings";
  ProgramPoint a,b;b.seconds=.2;p.points={a,b};return p;
}
}
TEST(MotionProgram, NoMissingDriverOverrideByDefault) {
  Harness h;h.bus->absent_address=6;h.ready();enable_axes(h);
  EXPECT_THROW(h.manager->prepare_program(hold(h),h.now),std::runtime_error);
  EXPECT_EQ(h.bus->counts[0x4b],0u);
}
TEST(MotionProgram, PartialProgramBarrierAndIndependentLocalCompletion) {
  Harness h;h.bus->absent_address=6;h.ready();enable_axes(h);
  auto s=h.manager->prepare_program(hold(h,true),h.now);
  EXPECT_FALSE(s.tcp_verified);EXPECT_FALSE(s.participating[5]);
  ASSERT_TRUE(h.manager->execute_program(s.program_id,"once",h.now));
  h.manager->renew_source_lease(Source::none,h.now); // Brak klienta nie anuluje zatwierdzonego programu.
  h.run(800);
  s=h.manager->program_status();EXPECT_EQ(s.state,"succeeded") << s.message;
  EXPECT_EQ(h.bus->counts[0x4b],1u);EXPECT_EQ(h.bus->counts[0xf5],5u);
  EXPECT_EQ(h.bus->prepared_payloads.count(6),0u);
  EXPECT_TRUE(h.manager->execute_program(s.program_id,"once",h.now));
  h.run(50);EXPECT_EQ(h.bus->counts[0x4b],1u);
}
TEST(MotionProgram, MissingParticipatingAckAndStopInvalidateProgram) {
  Harness h;h.ready();enable_axes(h);
  auto s=h.manager->prepare_program(hold(h),h.now);h.bus->missing_ack=3;
  ASSERT_TRUE(h.manager->execute_program(s.program_id,"ack",h.now));h.run(600);
  EXPECT_EQ(h.manager->program_status().state,"failed");EXPECT_EQ(h.bus->counts[0x4b],0u);
  EXPECT_EQ(h.bus->counts[0xf7],12u); // Sześć przygotowań ENABLE i sześć STOP.
  EXPECT_FALSE(h.manager->execute_program(s.program_id,"retry",h.now));
}
TEST(MotionProgram, SettingsAndStartChangesRejectOldPreparedProgram) {
  Harness h;h.ready();enable_axes(h);
  auto s=h.manager->prepare_program(hold(h),h.now);
  h.manager->update_motion_settings("changed",true,h.now);
  EXPECT_FALSE(h.manager->execute_program(s.program_id,"old",h.now));
  s=h.manager->prepare_program(hold(h),h.now);
  h.bus->positions[1]=10000;h.run(40);
  EXPECT_FALSE(h.manager->execute_program(s.program_id,"moved",h.now));
}
TEST(MotionProgram, LostParticipantStillStopsWithPartialModeEnabled) {
  Harness h;h.bus->absent_address=6;h.ready();enable_axes(h);
  auto s=h.manager->prepare_program(hold(h,true),h.now);
  ASSERT_TRUE(h.manager->execute_program(s.program_id,"lost",h.now));h.run(250);
  h.bus->missing_position=2;h.run(150);
  EXPECT_EQ(h.manager->program_status().state,"failed");
}
TEST(MotionProgram, StopDuringArmingNeverSendsSync) {
  Harness h;h.ready();enable_axes(h);
  auto s=h.manager->prepare_program(hold(h),h.now);ASSERT_TRUE(h.manager->execute_program(s.program_id,"stop",h.now));
  h.run(5);h.manager->stop("STOP");h.run(200);
  EXPECT_EQ(h.manager->program_status().state,"failed");EXPECT_EQ(h.bus->counts[0x4b],0u);
}
TEST(MotionCompiler, RejectsInvalidAndAcceptsNativeHoldWithMissingAxes) {
  auto cfg=configured();std::array<uint16_t,6> steps;steps.fill(16);
  PreparedMotionProgram p;ProgramPoint a,b;b.seconds=.2;p.points={a,b};p.participating[0]=true;
  auto out=compile_motion_program(p,cfg.axes,steps,.02);
  EXPECT_EQ(out.mode,"native_ptp");EXPECT_FALSE(out.tcp_verified);EXPECT_TRUE(out.frames[0].payloads[1].empty());
  p.points[1].seconds=0;EXPECT_THROW(compile_motion_program(p,cfg.axes,steps,.02),std::invalid_argument);
  p.points[1].seconds=.2;p.points[1].positions[0]=3;EXPECT_THROW(compile_motion_program(p,cfg.axes,steps,.02),std::invalid_argument);
}
TEST(MotionCompiler, QuinticPtpFallsBackToCompleteOrderedStream) {
  auto cfg=configured();for(auto &a:cfg.axes) a.tracking_error=.01;
  std::array<uint16_t,6> steps;steps.fill(16);
  PreparedMotionProgram p;p.participating.fill(true);ProgramPoint a,b;b.seconds=1;b.positions.fill(.1);p.points={a,b};
  const auto out=compile_motion_program(p,cfg.axes,steps,.02);
  EXPECT_EQ(out.mode,"streamed");ASSERT_EQ(out.frames.size(),51u);
  for(unsigned k=0;k<out.frames.size();++k) { EXPECT_NEAR(out.frames[k].seconds,k*.02,1e-9);for(const auto &payload:out.frames[k].payloads) EXPECT_EQ(payload.size(),8u); }
}
TEST(MotionCompiler, DetectsBetweenPointOvershootEvenBelowOneMillisecond) {
  auto cfg=configured();std::array<uint16_t,6> steps;steps.fill(16);
  PreparedMotionProgram p;p.participating[0]=true;ProgramPoint a,b;b.seconds=.0001;
  a.velocities[0]=b.velocities[0]=10000;p.points={a,b};
  EXPECT_THROW(compile_motion_program(p,cfg.axes,steps,.02),std::invalid_argument);
}
TEST(MotionProgram, StreamsAllSamplesInOrderWithoutBrowserLease) {
  Harness h;h.ready();enable_axes(h);
  auto p=hold(h);p.points.back().seconds=2;
  auto middle=p.points.front();middle.seconds=1;middle.positions[0]=.1;
  p.points.insert(p.points.begin()+1,middle);
  const auto s=h.manager->prepare_program(p,h.now);ASSERT_EQ(s.mode,"streamed");
  ASSERT_TRUE(h.manager->execute_program(s.program_id,"stream",h.now));
  h.run(2500);
  EXPECT_EQ(h.manager->program_status().state,"succeeded") << h.manager->program_status().message;
  EXPECT_EQ(h.bus->counts[0x4b],101u);EXPECT_EQ(h.bus->counts[0xf5],606u);
}
TEST(MotionProgram, MissingAckDuringStreamNeverCatchesUp) {
  Harness h;h.ready();enable_axes(h);
  auto p=hold(h);p.points.back().seconds=2;auto middle=p.points.front();middle.seconds=1;middle.positions[0]=.1;p.points.insert(p.points.begin()+1,middle);
  const auto s=h.manager->prepare_program(p,h.now);ASSERT_EQ(s.mode,"streamed");
  ASSERT_TRUE(h.manager->execute_program(s.program_id,"stream-stop",h.now));h.run(250);
  ASSERT_GT(h.bus->counts[0x4b],1u);h.bus->missing_ack=2;h.run(100);
  ASSERT_EQ(h.manager->program_status().state,"failed");const auto commits=h.bus->counts[0x4b];
  h.run(300);EXPECT_EQ(h.bus->counts[0x4b],commits);
}
TEST(MotionProgram, LateTickDoesNotSendOverdueSync) {
  Harness h;h.ready();enable_axes(h);
  auto s=h.manager->prepare_program(hold(h),h.now);ASSERT_TRUE(h.manager->execute_program(s.program_id,"late",h.now));
  for(unsigned i=0;i<250 && h.bus->counts[0xf5]<6;++i) h.run(1);
  ASSERT_EQ(h.bus->counts[0xf5],6u);h.run(2);ASSERT_EQ(h.bus->counts[0x4b],0u);
  h.now+=25ms;h.run(1);
  EXPECT_EQ(h.manager->program_status().state,"failed");EXPECT_EQ(h.bus->counts[0x4b],0u);
}
TEST(MotionProgram, OverrideDoesNotSkipPresentDisabledAxisOrModelChange) {
  Harness h;h.ready();enable_axes(h);
  auto s=h.manager->prepare_program(hold(h,true),h.now);
  h.manager->update_motion_model("different");EXPECT_EQ(h.manager->program_status().state,"invalidated");
  EXPECT_FALSE(h.manager->execute_program(s.program_id,"wrong-model",h.now));
  ASSERT_TRUE(h.manager->manual().set_enable(2,false,h.now));h.run(150);
  EXPECT_THROW(h.manager->prepare_program(hold(h,true),h.now),std::runtime_error);
}
TEST(MotionCompiler, NativePtpFitsQuantizedSynchronizedMovingProfiles) {
  auto cfg=configured();for(auto &axis:cfg.axes) { axis.gear_ratio=100;axis.tracking_error=.02;axis.max_acceleration=10; }
  std::array<uint16_t,6> steps;steps.fill(16);PreparedMotionProgram p;p.participating.fill(true);
  for(unsigned k=0;k<=45;++k) {
    ProgramPoint point;const double t=k*.02;point.seconds=t;
    for(unsigned j=0;j<6;++j) {
      const double factor=(j+1)/6.;
      if(t<.2) { point.positions[j]=.5*t*t*factor;point.velocities[j]=t*factor;point.accelerations[j]=factor; }
      else if(t<.7) { point.positions[j]=(.02+.2*(t-.2))*factor;point.velocities[j]=.2*factor; }
      else { point.positions[j]=(.14-.5*(.9-t)*(.9-t))*factor;point.velocities[j]=(.9-t)*factor;point.accelerations[j]=k<45 ? -factor : 0; }
    }
    p.points.push_back(point);
  }
  const auto compiled=compile_motion_program(p,cfg.axes,steps,.02);
  EXPECT_EQ(compiled.mode,"native_ptp");ASSERT_EQ(compiled.frames.size(),1u);
  EXPECT_NE(compiled.frames[0].payloads[0],compiled.frames[0].payloads[5]);
}
TEST(MotionProgram, NextPtpDoesNotRequireTogglingAllEnablesAgain) {
  Harness h;h.ready();enable_axes(h);
  auto s=h.manager->prepare_program(hold(h),h.now);
  ASSERT_TRUE(h.manager->execute_program(s.program_id,"first",h.now));h.run(800);
  ASSERT_EQ(h.manager->program_status().state,"succeeded");
  s=h.manager->prepare_program(hold(h),h.now);
  ASSERT_TRUE(h.manager->execute_program(s.program_id,"second",h.now));h.run(800);
  EXPECT_EQ(h.manager->program_status().state,"succeeded");EXPECT_EQ(h.bus->counts[0x4b],2u);
  EXPECT_FALSE(h.manager->manual().snapshot(h.now).axes[0].can_jog);
}

TEST(MotionProgram, PreviousCompletionCannotConsumeNextStageAck) {
  Harness h;h.ready();enable_axes(h);h.bus->complete_before_ack=true;
  auto p=hold(h);p.points.back().seconds=2;auto middle=p.points.front();middle.seconds=1;middle.positions[0]=.1;p.points.insert(p.points.begin()+1,middle);
  const auto s=h.manager->prepare_program(p,h.now);ASSERT_EQ(s.mode,"streamed");
  ASSERT_TRUE(h.manager->execute_program(s.program_id,"completion",h.now));h.run(2500);
  EXPECT_EQ(h.manager->program_status().state,"succeeded") << h.manager->program_status().message;
  EXPECT_EQ(h.bus->counts[0x4b],101u);EXPECT_GE(h.manager->snapshot().statistics.late,606u);
  EXPECT_LE(h.manager->snapshot().statistics.scheduled_utilization,.5);
}

TEST(MotionProgram, ZeroAtSameEncoderPositionStillInvalidatesPreparedCalibration) {
  Harness h;h.ready();enable_axes(h);
  const auto s=h.manager->prepare_program(hold(h),h.now);
  ASSERT_TRUE(h.manager->manual().zero(1,h.now));h.run(200);
  ASSERT_FALSE(h.manager->manual().calibration_valid(1));
  EXPECT_FALSE(h.manager->execute_program(s.program_id,"zero-invalid",h.now));
  EXPECT_EQ(h.bus->counts[0x4b],0u);
}
