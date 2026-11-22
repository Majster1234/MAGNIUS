#include <gtest/gtest.h>
#include "streaming_fixture.hpp"
using namespace streaming_test;
namespace {
BusConfig startup_config() {auto c=config();c.drive_startup=true;return c;}
unsigned count(const Port &port,unsigned axis,std::uint8_t code,int first=-1) {
  unsigned n=0;
  for(const auto &e:port.events) n+=e.axis==axis && e.data[0]==code && (first<0 || e.data[1]==first);
  return n;
}
const unsigned kAll=0x3f;
}

TEST(DriveStartup, DisabledByDefaultSoExistingOwnersKeepDriveState) {
  Rig rig;rig.ready();
  for(unsigned axis=1;axis<=6;++axis) EXPECT_EQ(count(*rig.port,axis,0xf3),0u);
  EXPECT_TRUE(rig.port->enabled[2]);EXPECT_FALSE(rig.bus->snapshot().startup_enabled);
}
TEST(DriveStartup, FirstCommandsDisableEveryAxisBeforeDiscovery) {
  Rig rig(startup_config());rig.run(2);
  for(unsigned axis=1;axis<=6;++axis) {EXPECT_FALSE(rig.port->enabled[axis]);EXPECT_EQ(count(*rig.port,axis,0xf3,0),1u);}
  EXPECT_EQ(rig.port->events.front().data[0],0xf3);
  EXPECT_EQ(rig.bus->snapshot().startup_pending,kAll);
}
TEST(DriveStartup, CompletesSeriesAndPublishesGeneration) {
  Rig rig(startup_config());rig.port->respond.fill(true);rig.run(1500);
  const auto s=rig.bus->snapshot();ASSERT_FALSE(s.fault)<<s.message;
  EXPECT_EQ(s.startup_pending,0u);EXPECT_EQ(s.startup_busy,0u);EXPECT_EQ(s.startup_ready,kAll);EXPECT_EQ(s.startup_generation,6u);
  for(unsigned axis=1;axis<=6;++axis) {
    EXPECT_EQ(count(*rig.port,axis,0xf7),1u);
    EXPECT_FALSE(rig.port->respond[axis]);   // 8C=00/00 przywrócone
    EXPECT_FALSE(rig.port->enabled[axis]);
    EXPECT_EQ(count(*rig.port,axis,0xf3,1),0u);
  }
  EXPECT_EQ(s.statistics.late,0u);
}
TEST(DriveStartup, ResponseModeIsNotRewrittenWhenAlreadyCorrect) {
  Rig rig(startup_config());rig.run(1500);
  for(unsigned axis=1;axis<=6;++axis) EXPECT_EQ(count(*rig.port,axis,0x8c),0u);
}
TEST(DriveStartup, SyncIsNeverOverwrittenSoLeftoverBufferStillFaults) {
  Rig rig(startup_config());rig.port->sync[3]=true;rig.run(1500);
  EXPECT_TRUE(rig.bus->snapshot().fault);EXPECT_FALSE(rig.port->enabled[3]);
  EXPECT_NE(rig.bus->recovery_state_name(),"idle");
}
TEST(DriveStartup, EnableIsBlockedUntilAxisFinishedInitialization) {
  Rig rig(startup_config());rig.port->refuse_disable=2;rig.run(1500);
  auto s=rig.bus->snapshot();EXPECT_EQ(s.startup_pending,2u);
  EXPECT_FALSE(rig.bus->manual().set_enable(2,true,rig.now));
  EXPECT_FALSE(rig.bus->manual().set_enable_all(true,rig.now));
  EXPECT_GT(count(*rig.port,2,0xf3,0),2u);       // F3=0 jest ponawiane
  rig.run(5000);s=rig.bus->snapshot();
  EXPECT_EQ(s.startup_failed,2u);EXPECT_EQ(s.startup_busy,0u);EXPECT_FALSE(s.fault);
  rig.port->refuse_disable=0;rig.run(1000);s=rig.bus->snapshot();
  EXPECT_EQ(s.startup_pending,0u);EXPECT_EQ(s.startup_failed,0u);EXPECT_EQ(s.startup_ready,kAll);
  rig.run(100);EXPECT_TRUE(rig.bus->manual().set_enable(2,true,rig.now));
}
TEST(DriveStartup, MissingDriveDoesNotHoldOthersAndIsInitializedWhenPoweredLater) {
  Rig rig(startup_config());rig.port->absent=5;rig.run(1500);
  auto s=rig.bus->snapshot();EXPECT_EQ(s.startup_ready,kAll&~16u);EXPECT_EQ(s.startup_busy,16u);
  rig.run(10000);s=rig.bus->snapshot();EXPECT_EQ(s.startup_busy,0u);EXPECT_EQ(s.startup_pending,16u);EXPECT_EQ(s.startup_failed,0u);
  rig.port->enabled[5]=true;rig.port->absent=0;rig.run(2500);s=rig.bus->snapshot();
  EXPECT_FALSE(rig.port->enabled[5]);EXPECT_EQ(s.startup_pending,0u);EXPECT_EQ(s.startup_ready,kAll);
  EXPECT_TRUE(s.devices[5].detected);
}
TEST(DriveStartup, DriveReturningAfterSilenceIsDisabledAgain) {
  Rig rig(startup_config());rig.run(1500);const auto generation=rig.bus->snapshot().startup_generation;
  rig.run(100);ASSERT_TRUE(rig.bus->manual().set_enable(4,true,rig.now));rig.run(800);ASSERT_TRUE(rig.port->enabled[4]);
  rig.port->absent=4;rig.port->rx.clear();rig.run(3000);
  EXPECT_EQ(rig.bus->snapshot().startup_ready&8u,0u);
  rig.port->absent=0;rig.run(2500);
  const auto s=rig.bus->snapshot();EXPECT_FALSE(rig.port->enabled[4]);EXPECT_EQ(s.startup_pending,0u);
  EXPECT_GT(s.startup_generation,generation);EXPECT_FALSE(rig.bus->manual().prepared(4));
}
TEST(DriveStartup, ShortDropoutKeepsEnabledAxisHolding) {
  Rig rig(startup_config());rig.run(1500);
  rig.run(100);ASSERT_TRUE(rig.bus->manual().set_enable(4,true,rig.now));rig.run(800);ASSERT_TRUE(rig.port->enabled[4]);
  const auto generation=rig.bus->snapshot().startup_generation;
  rig.port->absent=4;rig.port->rx.clear();rig.run(300);rig.port->absent=0;rig.run(1500);
  EXPECT_TRUE(rig.port->enabled[4]);EXPECT_EQ(rig.bus->snapshot().startup_generation,generation);
}
TEST(DriveStartup, ReconnectingTheSocketIsNotASystemStart) {
  Rig rig(startup_config());rig.run(1500);
  rig.run(100);ASSERT_TRUE(rig.bus->manual().set_enable(1,true,rig.now));rig.run(800);ASSERT_TRUE(rig.port->enabled[1]);
  rig.bus->close();rig.bus->reset_session(ResetMode::preserve_discovery);rig.port->rx.clear();
  rig.bus->open(rig.now);rig.run(1500);
  EXPECT_TRUE(rig.port->enabled[1]);EXPECT_EQ(rig.bus->snapshot().startup_pending,0u);
}
