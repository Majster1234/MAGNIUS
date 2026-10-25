#include <gtest/gtest.h>
#include <cmath>
#include "streaming_fixture.hpp"
using namespace streaming_test;
namespace {
RobotProfile changed_profile(const Rig &h,const std::string &revision,double ratio=3) {
  RobotProfile p;p.revision=revision;p.changed.fill(true);
  for(unsigned j=0;j<6;++j) {
    const auto &a=h.bus->config().axes[j];
    p.axes[j]={true,ratio,a.direction,.1,a.lower,a.upper,a.max_velocity,a.max_acceleration};
  }
  return p;
}
void disabled(Rig &h) {h.port->enabled.fill(false);h.run(350);}
void enabled(Rig &h) {h.port->enabled.fill(true);h.run(100);}
PreparedMotionProgram plan(Rig &h,const std::string &revision) {
  auto p=h.program(.2);p.revision=revision;return p;
}
void rejected(Rig &h,PreparedMotionProgram p,const std::string &reason) {
  try {h.bus->prepare_program(std::move(p),h.now);FAIL()<<"Przyjęto niedozwolony plan";}
  catch(const std::runtime_error &e) {EXPECT_NE(std::string(e.what()).find(reason),std::string::npos)<<e.what();}
}
}
TEST(ConfigurationRevision, LegacyYamlRevisionDoesNotPinCommittedProfile) {
  for(const std::string legacy:{"","obsolete"}) {
    auto c=config();c.program_revision=legacy;Rig h(c);h.ready();
    EXPECT_NO_THROW(h.bus->prepare_program(h.program(),h.now));
    EXPECT_EQ(h.bus->program_status().state,"ready");EXPECT_EQ(h.port->counts[0xf5],0u);
  }
}
TEST(ConfigurationRevision, EverySavedRatioUsesNewProfileWithoutRestartOrYamlEdit) {
  Rig h;h.ready();std::string previous="bench";
  for(double ratio:{1.,3.,5.,12.5,50.,.5}) {
    // Każda próba zaczyna się od tego samego położenia wałów emulatora.
    h.port->positions.fill(0);h.port->targets.fill(0);
    disabled(h);const auto revision="ratio-"+std::to_string(ratio);
    const auto p=changed_profile(h,revision,ratio);
    for(const auto phase:{ConfigurationPhase::prepare,ConfigurationPhase::commit,ConfigurationPhase::finish})
      ASSERT_TRUE(h.bus->configure(phase,revision,previous,p,h.now).accepted);
    enabled(h);const auto program=plan(h,revision);
    ASSERT_NO_THROW(h.execute(program,revision));h.run(1500);
    EXPECT_EQ(h.bus->program_status().state,"succeeded")<<h.bus->program_status().message;
    for(unsigned j=0;j<6;++j) {
      const auto &a=p.axes[j];
      EXPECT_DOUBLE_EQ(h.bus->config().axes[j].gear_ratio,ratio);
      EXPECT_NEAR(h.port->targets[j+1],a.direction*(.2-a.offset)*ratio*16384/(2*std::acos(-1.)),1.);
    }
    previous=revision;
  }
}
TEST(ConfigurationRevision, PreparedAndCommittedTransactionsBlockPreparationUntilFinish) {
  Rig h;h.ready();disabled(h);const auto p=changed_profile(h,"new");
  ASSERT_TRUE(h.bus->configure(ConfigurationPhase::prepare,"change","bench",p,h.now).accepted);
  rejected(h,plan(h,"bench"),"nie jest uzgodniona");
  ASSERT_TRUE(h.bus->configure(ConfigurationPhase::commit,"change","bench",p,h.now).accepted);
  rejected(h,plan(h,"new"),"nie jest uzgodniona");
  ASSERT_TRUE(h.bus->configure(ConfigurationPhase::finish,"change","bench",p,h.now).accepted);
  enabled(h);EXPECT_NO_THROW(h.bus->prepare_program(plan(h,"new"),h.now));
  EXPECT_EQ(h.port->counts[0xf5],0u);
}
TEST(ConfigurationRevision, SuccessfulSaveRejectsOldPlanAndInvalidatesPreparedProgram) {
  Rig h;h.ready();const auto old_plan=h.program();const auto old=h.bus->prepare_program(old_plan,h.now);
  disabled(h);const auto p=changed_profile(h,"new");
  ASSERT_TRUE(h.bus->configure(ConfigurationPhase::prepare,"change","bench",p,h.now).accepted);
  EXPECT_EQ(h.bus->program_status().state,"invalidated");
  for(const auto phase:{ConfigurationPhase::commit,ConfigurationPhase::finish})
    ASSERT_TRUE(h.bus->configure(phase,"change","bench",p,h.now).accepted);
  enabled(h);h.bus->update_motion_settings("settings",false,h.now);
  EXPECT_FALSE(h.bus->execute_program(old.program_id,"stale",h.now));
  rejected(h,old_plan,"Nieaktualna konfiguracja");
  EXPECT_NO_THROW(h.bus->prepare_program(plan(h,"new"),h.now));
  EXPECT_EQ(h.port->counts[0xf5],0u);
}
TEST(ConfigurationRevision, RollbackRequiresRepreparationAndRestoresPreviousRatio) {
  Rig h;h.ready();const auto old=h.bus->prepare_program(h.program(),h.now);
  disabled(h);const auto p=changed_profile(h,"new");
  for(const auto phase:{ConfigurationPhase::prepare,ConfigurationPhase::commit,ConfigurationPhase::rollback,ConfigurationPhase::finish})
    ASSERT_TRUE(h.bus->configure(phase,"change","bench",p,h.now).accepted);
  enabled(h);h.bus->update_motion_settings("settings",false,h.now);
  EXPECT_EQ(h.bus->program_status().state,"invalidated");
  EXPECT_FALSE(h.bus->execute_program(old.program_id,"stale",h.now));
  EXPECT_DOUBLE_EQ(h.bus->config().axes[0].gear_ratio,10.);
  rejected(h,plan(h,"new"),"Nieaktualna konfiguracja");
  EXPECT_NO_THROW(h.bus->prepare_program(plan(h,"bench"),h.now));
  EXPECT_EQ(h.port->counts[0xf5],0u);
}
TEST(ConfigurationRevision, BootstrapGateBlocksPreparationAndInvalidatesOldProgram) {
  Rig h;h.ready();const auto old=h.bus->prepare_program(h.program(),h.now);
  h.bus->require_configuration();
  EXPECT_EQ(h.bus->program_status().state,"invalidated");
  EXPECT_FALSE(h.bus->execute_program(old.program_id,"stale",h.now));
  rejected(h,h.program(),"nie jest uzgodniona");
  EXPECT_EQ(h.port->counts[0xf5],0u);
}
TEST(ConfigurationRevision, ExecutionEnableAndInvalidCalibrationRemainRequired) {
  auto c=config();c.program_enabled=false;Rig disabled_bus(c);disabled_bus.ready();
  rejected(disabled_bus,disabled_bus.program(),"Wykonanie PTP/LIN wyłączone");
  Rig h;h.ready();h.bus->require_configuration();auto p=changed_profile(h,"uncalibrated");
  p.calibration_invalid[0]=true;
  ASSERT_TRUE(h.bus->configure(ConfigurationPhase::bootstrap,"restart","",p,h.now).accepted);
  ASSERT_TRUE(h.bus->configure(ConfigurationPhase::finish,"restart","",p,h.now).accepted);
  h.ready();
  EXPECT_FALSE(h.bus->manual().calibration_valid(1));
  rejected(h,plan(h,"uncalibrated"),"kalibracja");EXPECT_EQ(h.port->counts[0xf5],0u);
}
