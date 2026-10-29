#include <gtest/gtest.h>
#include <fstream>
#include <iterator>
#include <cmath>
#include "robot_motion_planner/planner.hpp"
#include "../../mks_servo_can/test/streaming_fixture.hpp"
using namespace robot_motion_planner;
using namespace streaming_test;
namespace {
struct LinExecutor : ::testing::TestWithParam<bool> {};
}
TEST_P(LinExecutor, PhysicalLinStreamsRealPlannerTrajectoryWithSignedTargetsAndOffsets) {
  Context c;std::ifstream input(DEMO_URDF);c.urdf={std::istreambuf_iterator<char>(input),{}};
  c.model_id=robot_kinematics::model_id(c.urdf);c.configuration_revision="bench";
  c.seed={.2,.3,-.5,.2,.8,-.1};c.known.fill(true);c.lower.fill(-3);c.upper.fill(3);
  c.max_velocity.fill(1);c.max_acceleration.fill(2);c.allow_missing_execution=GetParam();
  auto cfg=config();
  for(unsigned j=0;j<6;++j) {
    cfg.axes[j]={static_cast<uint16_t>(j+1),double(j+2),j%2 ? -1. : 1.,c.seed[j],-3,3,1,2,.2,135};
    auto &p=c.profiles[j];p.configured=true;p.gear_ratio=cfg.axes[j].gear_ratio;p.direction=cfg.axes[j].direction;
    p.zero_offset_rad=c.seed[j];p.lower_rad=-3;p.upper_rad=3;p.velocity_rad_s=1;p.acceleration_rad_s2=2;
  }
  Rig h(cfg);
  if(GetParam()) {h.port->absent=4;h.port->extra_absent=6;c.simulated_start[3]=c.simulated_start[5]=true;}
  h.ready();auto target=c.seed;target[0]+=.005;target[1]+=.003;
  std::array<int64_t,6> codes;codes.fill(135);
  const auto plan=compute_physical_motion(c,target,1,codes,{}, {},TcpLimits{.01,.1,.2});
  mks_servo_can::PreparedMotionProgram program;
  program.id=plan.id;program.model_id=plan.model_id;program.revision=plan.configuration_revision;program.settings_revision="settings";
  for(const auto &point:plan.trajectory.points) {
    mks_servo_can::ProgramPoint p;p.seconds=point.time_from_start.sec+point.time_from_start.nanosec*1e-9;
    std::copy(point.positions.begin(),point.positions.end(),p.positions.begin());
    std::copy(point.velocities.begin(),point.velocities.end(),p.velocities.begin());
    std::copy(point.accelerations.begin(),point.accelerations.end(),p.accelerations.begin());program.points.push_back(p);
  }
  h.bus->update_motion_model(plan.model_id);h.bus->update_motion_settings("settings",GetParam(),h.now);
  auto stale=program;stale.revision="stale";EXPECT_THROW(h.bus->prepare_program(stale,h.now),std::runtime_error);
  h.port->events.clear();const auto ready=h.bus->prepare_program(program,h.now);
  EXPECT_EQ(h.port->counts[0xf5],0u);EXPECT_EQ(ready.tcp_verified,!GetParam());
  ASSERT_TRUE(h.bus->execute_program(ready.program_id,"lin",h.now));
  h.run(static_cast<unsigned>(std::ceil(ready.duration*1000))+800);
  ASSERT_EQ(h.bus->program_status().state,"succeeded")<<h.bus->program_status().message;
  EXPECT_EQ(h.port->counts[0x4b],0u);
  for(unsigned j=0;j<6;++j) if(ready.participating[j]) {
    const auto &a=cfg.axes[j];
    EXPECT_NEAR(h.port->targets[j+1],a.direction*(target[j]-a.offset)*a.gear_ratio*16384/(2*std::acos(-1.)),1.);
  }
  for(const auto &event:h.port->events) if(event.data[0]==0xf5) EXPECT_TRUE(ready.participating[event.axis-1]);
  // Drugi program sprawdza STOP tej samej ścieżki wykonania.
  h.bus->update_motion_settings("settings",GetParam(),h.now);
  const auto total=program.points.back().seconds;
  for(auto &point:program.points) {point.seconds=total-point.seconds;for(auto &v:point.velocities) v=-v;}
  std::reverse(program.points.begin(),program.points.end());
  const auto next=h.bus->prepare_program(program,h.now);ASSERT_TRUE(h.bus->execute_program(next.program_id,"stop-lin",h.now));
  h.run(200);h.bus->stop("STOP testu LIN");h.run(100);
  EXPECT_FALSE(h.bus->program_status().busy);EXPECT_GT(h.port->counts[0xf7],0u);
}
INSTANTIATE_TEST_SUITE_P(FullAndPartial,LinExecutor,::testing::Bool());
