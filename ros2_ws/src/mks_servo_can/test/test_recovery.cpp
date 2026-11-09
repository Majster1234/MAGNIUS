#include <gtest/gtest.h>
#include "streaming_fixture.hpp"
using namespace streaming_test;
namespace {
BusConfig accepted_config() {auto c=config();c.program_staged_reset_verified=true;return c;}
bool is_read(const Port::Event &e) {
  return e.data[0]==0 || e.data[0]==0x40 || e.data[0]==0x31 || e.data[0]==0x32 ||
      e.data[0]==0x3a || e.data[0]==0x3e || e.data[0]==0xf1;
}
}
TEST(Recovery, StopAtEveryRecoveryPhaseRestartsWithoutEnablingOrSync) {
  for(const std::string phase:{"stopping","verifying_disabled","overwriting","restoring","verifying","idle_disabled"}) {
    Rig h(accepted_config());h.ready();h.port->sync.fill(true);h.bus->stop("STOP",false);
    for(unsigned i=0;i<4000 && h.bus->recovery_state_name()!=phase;++i) h.run(1);
    ASSERT_EQ(h.bus->recovery_state_name(),phase)<<h.bus->snapshot().message;
    const auto first=h.port->events.size();h.bus->stop("Ponowny STOP",false);h.run(4000);
    EXPECT_EQ(h.bus->recovery_state_name(),"idle_disabled")<<h.bus->snapshot().message;
    for(auto it=h.port->events.begin()+first;it!=h.port->events.end();++it) {
      EXPECT_NE(it->data[0],0x4b);
      EXPECT_FALSE(it->data[0]==0xf3 && it->data[1]!=0);
    }
  }
}
TEST(Recovery, NeutralStopDoesNotReplaceMeasuredPositionWithZero) {
  Rig h(accepted_config());h.ready();
  for(unsigned j=1;j<=6;++j) h.port->positions[j]=1000*j;
  h.bus->stop("STOP",false);h.run(4000);
  ASSERT_EQ(h.bus->recovery_state_name(),"idle_disabled")<<h.bus->snapshot().message;
  for(unsigned j=1;j<=6;++j) EXPECT_EQ(h.port->positions[j],1000*j);
}
TEST(Recovery, StopPhasesHaveGuardGapsAndRepeatedStopCannotPostponeDisable) {
  Rig h(accepted_config());h.ready();
  const auto begin=h.port->events.size();const auto start=h.port->now;
  h.bus->stop("STOP",false);
  EXPECT_FALSE(h.bus->reset_allowed());
  for(unsigned i=0;i<19 && h.bus->snapshot().statistics.stop_pending;++i) {h.bus->stop("ponowny STOP",false);h.run(1);}
  unsigned counts[3]={};
  for(auto i=begin;i<h.port->events.size();++i) {
    const auto &e=h.port->events[i];const auto opcode=e.data[0];
    if(opcode==0xf7) {++counts[0];EXPECT_EQ(e.stamp,start);}
    else if(opcode==0xf5) {++counts[1];EXPECT_GE(e.stamp,start+6ms);EXPECT_LT(e.stamp,start+7ms);}
    else if(opcode==0xf3) {++counts[2];EXPECT_EQ(e.data[1],0);EXPECT_GE(e.stamp,start+12ms);EXPECT_LT(e.stamp,start+13ms);}
    else EXPECT_GE(e.stamp,start+18ms)<<"Zapytanie diagnostyczne w trakcie serii STOP";
  }
  EXPECT_EQ(counts[0],6u);EXPECT_EQ(counts[1],6u);EXPECT_EQ(counts[2],6u);
  EXPECT_FALSE(h.bus->snapshot().statistics.stop_pending);
}
TEST(Recovery, FirmwareRejectingDisabledF5CannotEnableOrTriggerTheOldBuffer) {
  Rig h(accepted_config());h.ready();h.port->sync.fill(true);h.port->enabled.fill(false);h.port->reject_disabled_f5=true;
  h.port->f7_clears_pending=false;h.port->pending_targets_valid[3]=true;h.port->pending_targets[3]=12345;
  h.bus->stop("STOP",false);h.run(4000);
  EXPECT_EQ(h.bus->recovery_state_name(),"recovery_failed");EXPECT_TRUE(h.bus->snapshot().fault);
  EXPECT_EQ(h.port->counts[0x4b],0u);EXPECT_TRUE(h.port->pending_targets_valid[3]);
  EXPECT_EQ(h.port->pending_targets[3],12345);
  for(unsigned j=1;j<=6;++j) {EXPECT_FALSE(h.port->enabled[j]);EXPECT_EQ(h.port->positions[j],0);}
}
TEST(Recovery, MissingAndLateNeutralAckKeepTheRecoveryFailedWithLiveReads) {
  for(bool late:{false,true}) {
    Rig h(accepted_config());h.ready();h.port->sync.fill(true);
    h.bus->stop("STOP",false);
    for(unsigned i=0;i<2000 && h.bus->recovery_state_name()!="overwriting";++i) h.run(1);
    ASSERT_EQ(h.bus->recovery_state_name(),"overwriting");
    if(late) h.port->axis_ack_delay[1]=200ms;else h.port->missing_ack=1;
    h.run(2000);
    EXPECT_EQ(h.bus->recovery_state_name(),"recovery_failed");EXPECT_EQ(h.port->counts[0x4b],0u);
    EXPECT_TRUE(h.bus->manual().snapshot(h.now).axes[5].position.valid);
  }
}
TEST(Recovery, WatchdogPauseIsOncePerUnconfirmedAxisAndOtherAxesKeepReading) {
  Rig h(accepted_config());h.ready();h.port->refuse_disable=1;
  const auto begin=h.now;h.bus->stop("STOP",false);h.run(5500);
  ASSERT_EQ(h.bus->recovery_state_name(),"recovery_failed");
  bool other=false,resumed=false;
  for(const auto &e:h.port->events) if(e.stamp>=begin+5000ms && is_read(e)) {
    if(e.axis==1) {EXPECT_GE(e.stamp,begin+5350ms);resumed=true;}
    else if(e.stamp<begin+5350ms) other=true;
  }
  EXPECT_TRUE(other);EXPECT_TRUE(resumed);EXPECT_TRUE(h.bus->snapshot().fault);
  const auto stops=h.port->counts[0xf7];h.run(1000);EXPECT_EQ(h.port->counts[0xf7],stops);
}
TEST(Recovery, DisabledWatchdogLeavesExplicitReasonWithoutAssumingRest) {
  Rig h(accepted_config());h.ready();h.port->watchdog=0;h.port->refuse_disable=1;
  h.bus->stop("STOP",false);h.run(5500);
  EXPECT_EQ(h.bus->recovery_state_name(),"recovery_failed");
  EXPECT_NE(h.bus->recovery_message().find("brak świeżego niezerowego watchdogu"),std::string::npos);
  EXPECT_TRUE(h.bus->manual().snapshot(h.now).axes[1].position.valid);
}
TEST(Recovery, LostAxisIsProbedAtMostOncePerSecondAfterTimeouts) {
  Rig h(accepted_config());h.ready();h.port->absent=3;
  h.bus->stop("STOP",false);h.run(7000);
  EXPECT_TRUE(h.bus->snapshot().fault);EXPECT_EQ(h.bus->recovery_state_name(),"recovery_failed");
  unsigned probes=0;
  for(const auto &e:h.port->events) if(e.axis==3 && is_read(e) && e.stamp>h.now-2s) ++probes;
  EXPECT_LE(probes,3u);EXPECT_TRUE(h.bus->manual().snapshot(h.now).axes[0].position.valid);
}
TEST(Recovery, ReconnectFollowsBackoffAndDoesNotEnableOrResume) {
  Rig h(accepted_config());h.ready();h.port->disconnected=true;h.run(15000);
  const auto &times=h.port->open_attempts;ASSERT_GE(times.size(),6u);
  EXPECT_GE(times[2]-times[1],1000ms);EXPECT_GE(times[3]-times[2],2000ms);
  EXPECT_GE(times[4]-times[3],5000ms);EXPECT_GE(times[5]-times[4],5000ms);
  h.port->disconnected=false;h.run(8000);
  EXPECT_TRUE(h.bus->snapshot().open);EXPECT_EQ(h.bus->recovery_state_name(),"idle_disabled");
  EXPECT_EQ(h.port->counts[0x4b],0u);for(unsigned j=1;j<=6;++j) EXPECT_FALSE(h.port->enabled[j]);
}
TEST(Recovery, OldSourceGenerationCannotSubmitEvenWithANewTimestampAfterReenable) {
  auto c=accepted_config();c.motion_enabled=true;Rig h(c);h.ready();
  ASSERT_TRUE(h.bus->activate(Source::trajectory,h.now));
  CommandBatch old;old.source=Source::trajectory;old.sequence=999;old.velocities_valid=true;
  old.configuration_generation=h.bus->control_state().configuration_generation;
  old.control_generation=h.bus->control_state().control_generation;
  h.bus->stop("STOP",false);h.run(4000);
  ASSERT_EQ(h.bus->recovery_state_name(),"idle_disabled");
  ASSERT_TRUE(h.bus->manual().set_enable_all(true,h.now));h.run(3000);
  ASSERT_TRUE(h.bus->activate(Source::trajectory,h.now));
  old.created=h.now;old.expires=h.now+20ms;
  EXPECT_FALSE(h.bus->submit(old));
  old.control_generation=h.bus->control_state().control_generation;
  EXPECT_TRUE(h.bus->submit(old));
}
TEST(Recovery, CorruptNeutralAckCannotReplaceTheRequiredValidAcknowledgement) {
  Rig h(accepted_config());h.ready();h.port->sync.fill(true);h.port->missing_ack=1;
  h.bus->stop("STOP",false);
  for(unsigned i=0;i<3000 && h.bus->recovery_state_name()!="overwriting";++i) h.run(1);
  ASSERT_EQ(h.bus->recovery_state_name(),"overwriting");
  const auto initial=h.port->counts[0xf5];
  for(unsigned i=0;i<3000 && h.port->counts[0xf5]==initial;++i) h.run(1);
  ASSERT_GT(h.port->counts[0xf5],initial);
  const auto invalid=h.bus->snapshot().statistics.invalid;
  h.port->rx.push_back({1,{0xf5,5,0},false,0,h.now+1ms});h.run(1000);
  EXPECT_GT(h.bus->snapshot().statistics.invalid,invalid);
  EXPECT_EQ(h.bus->recovery_state_name(),"recovery_failed");EXPECT_EQ(h.port->counts[0x4b],0u);
}
TEST(Recovery, InconsistentReadbackOfRestoredSettingsKeepsMotionBlocked) {
  for(bool response_mode:{false,true}) {
    Rig h(accepted_config());h.ready();h.port->sync.fill(true);h.bus->stop("STOP",false);
    for(unsigned i=0;i<3000 && h.bus->recovery_state_name()!="restoring";++i) h.run(1);
    ASSERT_EQ(h.bus->recovery_state_name(),"restoring");
    if(response_mode) h.port->bad_response_mode=true;else h.port->bad_sync_mode=true;
    h.run(1500);
    EXPECT_EQ(h.bus->recovery_state_name(),"recovery_failed");EXPECT_TRUE(h.bus->snapshot().fault);
    EXPECT_EQ(h.port->counts[0x4b],0u);for(unsigned j=1;j<=6;++j) EXPECT_FALSE(h.port->enabled[j]);
  }
}
