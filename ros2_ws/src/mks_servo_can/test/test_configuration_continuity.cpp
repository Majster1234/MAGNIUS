#include <gtest/gtest.h>
#include <cmath>
#include "streaming_fixture.hpp"
using namespace streaming_test;
namespace {
RobotProfile profile(Rig &h,const std::string &revision) {
  RobotProfile p;p.revision=revision;
  for(unsigned j=0;j<6;++j) {const auto &a=h.bus->config().axes[j];p.axes[j]={true,a.gear_ratio,a.direction,a.offset,a.lower,a.upper,a.max_velocity,a.max_acceleration};}
  return p;
}
void stopped(Rig &h) {h.port->enabled.fill(false);h.run(350);}
ConfigurationResult prepare(Rig &h,RobotProfile p,const std::string &op="change") {
  return h.bus->configure(ConfigurationPhase::prepare,op,h.bus->configure(ConfigurationPhase::status,"","",p,h.now).revision,p,h.now);
}
void finish(Rig &h,const RobotProfile &p,const std::string &op="change") {
  for(auto phase:{ConfigurationPhase::commit,ConfigurationPhase::finish}) ASSERT_TRUE(h.bus->configure(phase,op,"",p,h.now).accepted);
}
}
TEST(ConfigurationContinuity, NonzeroShaftsPreserveAnglesAcrossRepeatedRatiosAndBothDirections) {
  auto c=config();for(unsigned j=0;j<6;++j) {auto &a=c.axes[j];a.gear_ratio=1;a.direction=j%2 ? -1:1;a.offset=j%2 ? -.1:.2;}
  Rig h(c);h.ready();for(unsigned j=1;j<=6;++j) h.port->positions[j]=j%2 ? -4096:4096;stopped(h);
  const auto before=h.bus->control_state();const auto counts=h.port->counts;
  for(double ratio:{3.,1.,2.5,.5,12.5}) {
    auto p=profile(h,"ratio-"+std::to_string(ratio));for(auto &a:p.axes) a.gear_ratio=ratio;
    const auto op=p.revision;const auto prepared=prepare(h,p,op);ASSERT_TRUE(prepared.accepted)<<prepared.message;
    EXPECT_EQ(prepared.resolved_configuration.revision,p.revision);
    finish(h,p,op);
    const auto after=h.bus->control_state();
    for(unsigned j=0;j<6;++j) {
      EXPECT_NEAR(after.positions[j].value,before.positions[j].value,1e-12);
      const auto &a=h.bus->config().axes[j];
      EXPECT_EQ(CanBusManager::joint_to_position(a,after.positions[j].value),h.port->positions[j+1]);
      EXPECT_EQ(streaming_setpoint(a,after.positions[j].value,0).ticks,h.port->positions[j+1]);
      const auto next=CanBusManager::joint_to_position(a,after.positions[j].value+.01);
      EXPECT_NEAR(next-h.port->positions[j+1],a.direction*.01*ratio*16384/(2*std::acos(-1.)),.51);
      EXPECT_FALSE(prepared.resolved_configuration.calibration_invalid[j]);
    }
  }
  for(unsigned cmd:{0xf3u,0xf5u,0xf6u,0x92u}) EXPECT_EQ(h.port->counts[cmd],counts[cmd]);
}
TEST(ConfigurationContinuity, RejectsMixedReferenceChangesAndUnrepresentableLimits) {
  Rig h;h.ready();stopped(h);
  for(unsigned kind=0;kind<5;++kind) {
    auto p=profile(h,"reject");p.axes[0].gear_ratio=3;
    if(kind==0) p.axes[0].offset=.1;
    if(kind==1) p.axes[0].direction=-1;
    if(kind==2) p.confirm_calibration[0]=true;
    if(kind==3) p.axes[0].upper=100000;
    if(kind==4) p.axes[0].velocity=10000;
    const auto r=prepare(h,p);EXPECT_FALSE(r.accepted)<<kind;
  }
  h.port->positions[1]=4096;h.run(150);auto p=profile(h,"limits");p.axes[0].gear_ratio=3;p.axes[0].upper=.1;
  EXPECT_FALSE(prepare(h,p).accepted);
}
TEST(ConfigurationContinuity, CommitRejectsMovedLostEnabledOrResetAnchor) {
  for(unsigned kind=0;kind<4;++kind) {
    Rig h;h.ready();stopped(h);auto p=profile(h,"new");p.axes[0].gear_ratio=3;
    ASSERT_TRUE(prepare(h,p).accepted);
    if(kind==0) {h.port->positions[1]=4;h.run(160);}
    if(kind==1) {h.port->missing_position=1;h.run(150);}
    if(kind==2) {h.port->enabled[1]=true;h.run(120);}
    if(kind==3) {h.bus->close();h.bus->reset_session();h.bus->open(h.now);h.run(500);}
    EXPECT_FALSE(h.bus->configure(ConfigurationPhase::commit,"change","",p,h.now).accepted)<<kind;
    ASSERT_TRUE(h.bus->configure(ConfigurationPhase::rollback,"change","",p,h.now).accepted);
    ASSERT_TRUE(h.bus->configure(ConfigurationPhase::finish,"change","",p,h.now).accepted);
    EXPECT_DOUBLE_EQ(h.bus->config().axes[0].gear_ratio,10);
  }
}
TEST(ConfigurationContinuity, MissingAxisStaysUncalibratedUntilSelectiveConfirmationAndSurvivesBootstrap) {
  Rig h;h.ready();stopped(h);h.port->missing_position=1;h.port->extra_absent=2;h.run(1000);
  auto p=profile(h,"offline");p.axes[0].gear_ratio=3;p.axes[1].gear_ratio=3;
  const auto prepared=prepare(h,p);ASSERT_TRUE(prepared.accepted)<<prepared.message;finish(h,p);
  EXPECT_FALSE(h.bus->manual().calibration_valid(1));EXPECT_FALSE(h.bus->manual().calibration_valid(2));
  h.port->missing_position=0;h.port->extra_absent=0;h.run(500);EXPECT_FALSE(h.bus->manual().calibration_valid(1));
  auto restored=h.bus->configure(ConfigurationPhase::status,"","",p,h.now).resolved_configuration;
  h.bus->require_configuration();ASSERT_TRUE(h.bus->configure(ConfigurationPhase::bootstrap,"restart","",restored,h.now).accepted);
  ASSERT_TRUE(h.bus->configure(ConfigurationPhase::finish,"restart","",restored,h.now).accepted);h.run(800);
  EXPECT_FALSE(h.bus->manual().calibration_valid(1));
  auto confirmation=profile(h,"confirmed");confirmation.confirm_calibration[0]=true;
  const auto r=prepare(h,confirmation,"confirm");ASSERT_TRUE(r.accepted)<<r.message;finish(h,confirmation,"confirm");
  EXPECT_TRUE(h.bus->manual().calibration_valid(1));EXPECT_FALSE(h.bus->manual().calibration_valid(2));
}
TEST(ConfigurationContinuity, RollbackRestoresOffsetAndInvalidatesCommandGeneration) {
  Rig h;h.ready();h.port->positions[1]=4096;stopped(h);
  const auto before=h.bus->control_state();const auto old=h.bus->config().axes[0];auto p=profile(h,"new");p.axes[0].gear_ratio=3;
  ASSERT_TRUE(prepare(h,p).accepted);ASSERT_TRUE(h.bus->configure(ConfigurationPhase::commit,"change","",p,h.now).accepted);
  EXPECT_NE(h.bus->config().axes[0].offset,old.offset);
  ASSERT_TRUE(h.bus->configure(ConfigurationPhase::rollback,"change","",p,h.now).accepted);
  ASSERT_TRUE(h.bus->configure(ConfigurationPhase::finish,"change","",p,h.now).accepted);
  const auto after=h.bus->control_state();EXPECT_GT(after.configuration_generation,before.configuration_generation);
  EXPECT_DOUBLE_EQ(h.bus->config().axes[0].offset,old.offset);EXPECT_DOUBLE_EQ(after.positions[0].value,before.positions[0].value);
}
TEST(ConfigurationContinuity, PhysicalProgramStartsAtSameShaftCountAndUsesNewRatioForNextJointDelta) {
  auto c=config();for(auto &a:c.axes) a.gear_ratio=1;
  Rig h(c);h.ready();h.port->positions[1]=4096;stopped(h);
  const double start=h.bus->control_state().positions[0].value;
  auto p=profile(h,"rebased");p.axes[0].gear_ratio=3;
  ASSERT_TRUE(prepare(h,p).accepted);finish(h,p);
  h.port->enabled.fill(true);h.run(150);
  auto motion=h.program();motion.revision="rebased";
  for(unsigned j=0;j<6;++j) motion.points.back().positions[j]=motion.points.front().positions[j];
  motion.points.back().positions[0]=start+.01;
  const auto begin=h.port->events.size();h.execute(motion);h.run(1600);
  EXPECT_EQ(h.bus->program_status().state,"succeeded")<<h.bus->program_status().message;
  auto first=std::find_if(h.port->events.begin()+begin,h.port->events.end(),[](const auto &e){return e.axis==1 && e.data[0]==0xf5;});
  ASSERT_NE(first,h.port->events.end());
  EXPECT_EQ((first->data[4]<<16)|(first->data[5]<<8)|first->data[6],4096);
  EXPECT_EQ(h.port->targets[1],CanBusManager::joint_to_position(h.bus->config().axes[0],start+.01));
  EXPECT_NEAR(h.port->targets[1]-4096,.01*3*16384/(2*std::acos(-1.)),.51);
}
