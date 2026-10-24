#include <gtest/gtest.h>
#include <cmath>
#include "streaming_fixture.hpp"
#include <linux/can/error.h>
using namespace streaming_test;
TEST(MotionProgram, NoMissingDriverOverrideByDefault) {
  Rig h;h.port->absent=6;h.ready();EXPECT_THROW(h.bus->prepare_program(h.program(),h.now),std::runtime_error);
}
TEST(MotionProgram, FireAndForgetAllSixAxesRunAtHundredHertz) {
  Rig h;h.ready();h.execute(h.program());h.run(1500);
  EXPECT_EQ(h.bus->program_status().state,"succeeded")<<h.bus->program_status().message;
  EXPECT_EQ(h.port->counts[0xf5],606u);EXPECT_EQ(h.port->counts[0x4b],0u);
  std::vector<Time> times;for(const auto &e:h.port->events) if(e.axis==1 && e.data[0]==0xf5) times.push_back(e.stamp);
  ASSERT_EQ(times.size(),101u);for(unsigned i=1;i<times.size();++i) EXPECT_EQ(times[i]-times[i-1],10ms);
  EXPECT_FALSE(h.bus->snapshot().fault);
}
TEST(MotionProgram, FirstStreamSetPrecedesEncoderRequestsAndDiagnosticQueries) {
  Rig h;h.ready();h.port->events.clear();h.execute(h.program());h.run(300);
  const auto first=std::find_if(h.port->events.begin(),h.port->events.end(),[](const auto &e){return e.data[0]==0xf5;});
  ASSERT_NE(first,h.port->events.end());
  EXPECT_EQ(std::count_if(h.port->events.begin(),first,[](const auto &e){return e.data[0]==0x31;}),0);
  const auto query=std::find_if(first,h.port->events.end(),[](const auto &e){return e.data[0]==0x31;});
  ASSERT_NE(query,h.port->events.end());ASSERT_GE(std::distance(first,query),6);
  for(unsigned j=0;j<6;++j) EXPECT_EQ((first+j)->data[0],0xf5);
  EXPECT_EQ(h.port->counts[0x4b],0u);
}
TEST(MotionProgram, NoReadRequestsInLastFourMillisecondsBeforeNextSet) {
  Rig h;h.ready();h.port->delay=2ms;h.port->events.clear();h.execute(h.program());h.run(1600);
  const auto first=std::find_if(h.port->events.begin(),h.port->events.end(),[](const auto &e){return e.data[0]==0xf5;});
  const auto last=std::find_if(h.port->events.rbegin(),h.port->events.rend(),[](const auto &e){return e.data[0]==0xf5;});
  ASSERT_NE(first,h.port->events.end());ASSERT_NE(last,h.port->events.rend());
  for(const auto &e:h.port->events) if(e.stamp>=first->stamp && e.stamp<=last->stamp &&
      (e.data[0]==0x31 || e.data[0]==0x32 || e.data[0]==0xf1 || e.data[0]==0x3a || e.data[0]==0x3e))
    EXPECT_LT((e.stamp-first->stamp)%10ms,6ms);
  EXPECT_EQ(h.bus->program_status().state,"succeeded");
}
TEST(MotionProgram, IntermediateDeviationDoesNotGateAnyAxis) {
  Rig h;h.ready();h.port->frozen[1]=true;h.execute(h.program(.4,2));h.run(2300);
  EXPECT_EQ(h.port->counts[0xf5],1206u);EXPECT_EQ(h.bus->program_status().state,"waiting_position");
  EXPECT_FALSE(h.bus->snapshot().fault);
  const auto commands=h.port->counts[0xf5];h.run(2000);EXPECT_EQ(h.port->counts[0xf5],commands);
  h.port->frozen[1]=false;h.port->positions[1]=h.port->targets[1];h.run(400);
  EXPECT_EQ(h.bus->program_status().state,"succeeded");EXPECT_EQ(h.port->counts[0xf5],commands);
}
TEST(MotionProgram, MissingPositionNeverBlocksIntermediateStreamButBlocksFinalSuccess) {
  Rig h;h.ready();auto p=h.program(.4,1);h.execute(p);h.run(180);h.port->missing_position=2;h.run(1400);
  EXPECT_EQ(h.port->counts[0xf5],606u);EXPECT_EQ(h.bus->program_status().state,"waiting_feedback");
  EXPECT_FALSE(h.bus->snapshot().fault);h.port->missing_position=0;h.run(400);EXPECT_EQ(h.bus->program_status().state,"succeeded");
}
TEST(MotionProgram, PartialProgramCommandsOnlyDetectedParticipants) {
  Rig h;h.port->absent=4;h.ready();h.execute(h.program(.2,1,true));h.run(1500);
  EXPECT_EQ(h.bus->program_status().state,"succeeded");EXPECT_FALSE(h.bus->program_status().tcp_verified);
  for(const auto &e:h.port->events) if(e.data[0]==0xf5) EXPECT_NE(e.axis,4u);
  EXPECT_EQ(h.port->counts[0xf5],505u);
}
TEST(MotionProgram, SettingsModelAndStartChangesInvalidatePreparedProgram) {
  for(unsigned change=0;change<3;++change) {
    Rig h;h.ready();const auto s=h.bus->prepare_program(h.program(),h.now);
    if(change==0) h.bus->update_motion_settings("new",false,h.now);
    else if(change==1) h.bus->update_motion_model("new");
    else {h.port->positions[1]=1000;h.run(50);}
    EXPECT_FALSE(h.bus->execute_program(s.program_id,"old",h.now));EXPECT_EQ(h.port->counts[0xf5],0u);
  }
}
TEST(MotionProgram, SetupVerifiesNoResponseModeAndDisabledSynchronization) {
  for(unsigned broken=0;broken<3;++broken) {
    Rig h;h.ready();h.port->bad_response_mode=broken==0;h.port->bad_sync_mode=broken==1;h.port->bad_watchdog=broken==2;
    h.execute(h.program());h.run(600);EXPECT_TRUE(h.bus->snapshot().fault);EXPECT_EQ(h.port->counts[0xf5],0u);
  }
}
TEST(MotionProgram, LateIoTickDropsHistoryWithoutBurstOrPhaseDrift) {
  Rig h;h.ready();h.execute(h.program(.4,2));h.run(300);
  const auto count=h.port->counts[0xf5];h.now+=37ms;h.run(1);
  EXPECT_EQ(h.port->counts[0xf5],count+6);EXPECT_GT(h.bus->snapshot().statistics.dropped_commands,0u);
  h.run(2300);EXPECT_EQ(h.bus->program_status().state,"succeeded");
  std::vector<Time> times;for(const auto &e:h.port->events) if(e.axis==1 && e.data[0]==0xf5) times.push_back(e.stamp);
  for(unsigned i=1;i<times.size();++i) EXPECT_GE(times[i]-times[i-1],1ms);
}
TEST(MotionProgram, NonzeroSuccessDoesNotActivateOldJtcAndNextPtpWorks) {
  Rig h;h.ready();h.execute(h.program(.4,1));h.run(1500);ASSERT_EQ(h.bus->program_status().state,"succeeded");
  h.run(500);EXPECT_FALSE(h.bus->snapshot().fault);h.execute(h.program(0,1),"return");h.run(1500);
  EXPECT_EQ(h.bus->program_status().state,"succeeded");EXPECT_FALSE(h.bus->snapshot().fault);
}
TEST(MotionProgram, StopWorksDuringMotionOrFinalWaitWithoutAcceptanceFlags) {
  for(bool final:{false,true}) {
    Rig h;h.ready();h.port->frozen[1]=true;h.execute(h.program(.4,1));h.run(final ? 1500 : 400);
    const auto before=h.port->events.size();h.bus->stop("operator");h.run(100);
    EXPECT_TRUE(h.bus->snapshot().fault);ASSERT_GE(h.port->events.size(),before+12);
    for(unsigned j=0;j<6;++j) {EXPECT_EQ(h.port->events[before+j].data[0],0xf7);EXPECT_EQ(h.port->events[before+6+j].data[0],0xf6);}
    const auto tx=h.port->events.size();h.run(200);EXPECT_EQ(h.port->events.size(),tx);
  }
}
TEST(MotionProgram, BusOffTransportLossDisableAndStallAbortStream) {
  for(unsigned error=0;error<4;++error) {
    Rig h;h.ready();h.execute(h.program(.4,2));h.run(350);
    if(error==0) h.port->rx.push_back({0,{},true,CAN_ERR_BUSOFF,h.now});
    if(error==1) h.port->disconnected=true;
    if(error==2) h.port->enabled[1]=false;
    if(error==3) h.port->stalled=1;
    h.run(200);EXPECT_TRUE(h.bus->snapshot().fault);EXPECT_EQ(h.bus->program_status().state,"failed");
  }
}
TEST(MotionProgram, BudgetRejectsSixAxesAtFiveHundredButAcceptsFourWithReserve) {
  Rig six(config(500000));six.ready();EXPECT_THROW(six.bus->prepare_program(six.program(),six.now),std::runtime_error);
  Rig four(config(500000));four.port->absent=6;four.port->extra_absent=4;four.ready();
  EXPECT_NO_THROW(four.execute(four.program(.2,1,true)));four.run(1500);
  EXPECT_EQ(four.bus->program_status().state,"succeeded");
  EXPECT_NEAR(streaming_bus_utilization(6,1000000),.2658,1e-12);
  EXPECT_NEAR(streaming_bus_utilization(4,500000),.3544,1e-12);
  EXPECT_LE(streaming_bus_utilization(6,1000000)+kFutureModuleReservedUtilization,kTotalPlannedUtilization);
}
TEST(MotionProgram, FinalSuccessNeedsFreshSamplesSeparatedByHundredMilliseconds) {
  Rig h;h.ready();h.execute(h.program(.2,.5));
  for(unsigned t=0;t<3000 && h.port->counts[0xf5]<306 && !h.bus->snapshot().fault;++t) h.run(1);
  ASSERT_EQ(h.port->counts[0xf5],306u);
  ASSERT_FALSE(h.bus->snapshot().fault);h.run(50);EXPECT_TRUE(h.bus->program_status().busy);
  h.run(300);EXPECT_EQ(h.bus->program_status().state,"succeeded");
}
TEST(MotionProgram, UseAckAndInvalidPositionPeriodAreRejected) {
  Rig h;auto c=config();c.use_ack=true;EXPECT_THROW(h.bus->configure_program_authorization(c),std::invalid_argument);
  c.use_ack=false;c.program_position_period=100ms;EXPECT_THROW(h.bus->configure_program_authorization(c),std::invalid_argument);
  c.program_position_period=10ms;c.program_position_tolerance[0]=0;EXPECT_THROW(h.bus->configure_program_authorization(c),std::invalid_argument);
}
TEST(MotionCompiler, StartAndEndHoldDoNotStopCommandOrAddLookaheadAtZeroVelocity) {
  auto axes=config().axes;std::array<uint16_t,6> steps;steps.fill(16);PreparedMotionProgram p;p.participating.fill(true);
  ProgramPoint a,b;b.seconds=1;b.positions.fill(.4);p.points={a,b};p=compile_streaming_program(p,axes,steps);
  ASSERT_EQ(p.frames.size(),101u);EXPECT_EQ(p.mode,"streamed");EXPECT_EQ(p.frames[0].payloads[0][2],3);
  EXPECT_EQ(p.frames.back().payloads[0],absolute_position_request(1,CanBusManager::joint_to_position(axes[0],.4),3,240));
}
TEST(MotionCompiler, RejectsBetweenPointOvershootAndNonfiniteSamples) {
  auto axes=config().axes;std::array<uint16_t,6> steps;steps.fill(16);PreparedMotionProgram p;p.participating.fill(true);
  ProgramPoint a,b;b.seconds=.0001;a.positions.fill(1.999);b.positions.fill(1.999);a.accelerations.fill(1e10);p.points={a,b};
  EXPECT_THROW(compile_streaming_program(p,axes,steps),std::invalid_argument);
  p.points[0].accelerations.fill(0);p.points[1].positions[0]=NAN;EXPECT_THROW(compile_streaming_program(p,axes,steps),std::invalid_argument);
}
