#include <gtest/gtest.h>
#include <cmath>
#include "streaming_fixture.hpp"
using namespace streaming_test;
namespace {
constexpr double kBaseLookahead=.025,kFastLookahead=.05;
BusConfig schedule_config(bool enabled) {
  auto c=config();
  for(auto &s:c.program_streaming) {s.lookahead_time_sec=kBaseLookahead;s.position_kp=4;}
  c.program_gain_schedule={enabled,.2,kFastLookahead,12.};
  return c;
}
// Quintic start–stop: szczyt prędkości 1,875·droga/czas.
PreparedMotionProgram smooth(Rig &h,const std::string &type,double distance,double seconds) {
  auto p=h.program(distance,seconds);p.motion_type=type;const auto a=p.points.front();p.points.clear();
  for(unsigned k=0;k<=100;++k) {
    const double x=k/100.,s=x*x*x*(10+x*(-15+6*x)),ds=30*x*x*(x-1)*(x-1),dds=60*x*(1+x*(-3+2*x));ProgramPoint point;point.seconds=x*seconds;
    for(unsigned j=0;j<6;++j) {
      const double d=distance-a.positions[j];point.positions[j]=a.positions[j]+d*s;point.velocities[j]=d*ds/seconds;point.accelerations[j]=d*dds/(seconds*seconds);
    }
    p.points.push_back(point);
  }
  return p;
}
std::int32_t f5_ticks(const Rig &h,unsigned axis,std::size_t index) {
  std::size_t n=0;
  for(const auto &e:h.port->events) if(e.axis==axis && e.data[0]==0xf5 && n++==index) {
    std::int32_t v=(e.data[4]<<16)|(e.data[5]<<8)|e.data[6];return v&0x800000 ? v-0x1000000 : v;
  }
  ADD_FAILURE()<<"brak ramki F5";return 0;
}
std::int32_t run_mid_ticks(bool enabled,const std::string &type,double seconds=1.) {
  Rig h(schedule_config(enabled));h.ready();h.synchronized();h.execute(smooth(h,type,.2,seconds));h.run(static_cast<unsigned>(seconds*600));
  EXPECT_FALSE(h.bus->snapshot().fault)<<h.bus->snapshot().message;
  return f5_ticks(h,1,static_cast<std::size_t>(seconds*50));
}
double lead_ticks(double velocity,double seconds) {return velocity*seconds*10*16384/(2*std::acos(-1.));}
}
TEST(ProgramGains, OnlyFastPtpUsesFastSettings) {
  ProgramGainSchedule schedule{true,.2,.05,12.};PreparedMotionProgram p;p.participating.fill(true);ProgramPoint point;point.velocities[3]=-.25;p.points={point};
  p.motion_type="PTP";EXPECT_TRUE(program_uses_fast_gains(p,schedule));
  p.motion_type="LIN";EXPECT_FALSE(program_uses_fast_gains(p,schedule));
  p.motion_type="CHAIN";EXPECT_FALSE(program_uses_fast_gains(p,schedule));
  p.motion_type="PTP";p.participating[3]=false;EXPECT_FALSE(program_uses_fast_gains(p,schedule));
  p.participating[3]=true;p.points[0].velocities[3]=.19;EXPECT_FALSE(program_uses_fast_gains(p,schedule));
  p.points[0].velocities[3]=.25;schedule.enabled=false;EXPECT_FALSE(program_uses_fast_gains(p,schedule));
}
TEST(ProgramGains, FastPtpTargetLeadsByLongerLookahead) {
  const auto base=run_mid_ticks(false,"PTP"),fast=run_mid_ticks(true,"PTP");
  EXPECT_NEAR(fast-base,lead_ticks(.375,kFastLookahead-kBaseLookahead),40);
}
TEST(ProgramGains, LinAndSlowPtpKeepBaseSettings) {
  EXPECT_EQ(run_mid_ticks(true,"LIN"),run_mid_ticks(false,"LIN"));
  EXPECT_EQ(run_mid_ticks(true,"PTP",2.),run_mid_ticks(false,"PTP",2.));   // szczyt .1875 rad/s < próg .2
}
TEST(ProgramGains, PreparationReportsFastSettingsAndManualTuningDisablesThem) {
  Rig h(schedule_config(true));h.ready();h.synchronized();
  EXPECT_NE(h.bus->prepare_program(smooth(h,"PTP",.2,1.),h.now).message.find("szybkiego PTP"),std::string::npos);
  std::string reason;ExecutionTuning t{kBaseLookahead,1.,4.,2.,.01,-1,0.,false};
  ASSERT_TRUE(h.bus->set_execution_tuning(t,reason))<<reason;
  EXPECT_EQ(h.bus->prepare_program(smooth(h,"PTP",.2,1.),h.now).message.find("szybkiego PTP"),std::string::npos);
}
TEST(ProgramGains, TuningValidatesAccelerationTerm) {
  ExecutionTuning t{.025,1.,4.,2.,.01,-1,1.,true};EXPECT_TRUE(validate_execution_tuning(t).empty());
  t.acceleration_ff_scale=2.1;EXPECT_FALSE(validate_execution_tuning(t).empty());
}
namespace {
unsigned ramp_rpm_units(const std::string &type,double scale) {
  auto c=schedule_config(false);c.program_acceleration_ff=scale;Rig h(c);h.ready();h.synchronized();h.execute(smooth(h,type,.2,1.));h.run(600);
  EXPECT_FALSE(h.bus->snapshot().fault)<<h.bus->snapshot().message;
  std::size_t n=0;
  for(const auto &e:h.port->events) if(e.axis==1 && e.data[0]==0xf5 && n++==20) return ((e.data[1]&0x0fu)<<8)|e.data[2];
  ADD_FAILURE()<<"brak ramki F5";return 0;
}
}
TEST(ProgramGains, AccelerationTermAppliesOnlyToLin) {
  EXPECT_GT(ramp_rpm_units("LIN",1.),ramp_rpm_units("LIN",0.));
  EXPECT_EQ(ramp_rpm_units("PTP",1.),ramp_rpm_units("PTP",0.));
  EXPECT_EQ(ramp_rpm_units("CHAIN",1.),ramp_rpm_units("CHAIN",0.));
}
namespace {
struct HoldRun {unsigned sync{},f5{};std::vector<unsigned> hold_rpm;std::string state;};
HoldRun hold_run(unsigned period,double rpm) {
  Rig h(schedule_config(false));h.ready();h.synchronized();
  std::string reason;ExecutionTuning t{kBaseLookahead,1.,4.,2.,.01,-1,0.,false,period,rpm};
  EXPECT_TRUE(h.bus->set_execution_tuning(t,reason))<<reason;
  h.port->frozen[1]=true;   // J1 zostaje poza tolerancją, więc strumień trzymania trwa do limitu dolotu
  h.execute(smooth(h,"PTP",.05,.2));const auto frames=21u;h.run(1500);
  HoldRun out;out.state=h.bus->program_status().state;std::size_t n=0;
  for(const auto &e:h.port->events) {
    if(e.data[0]==0x4b) ++out.sync;
    if(e.axis==2 && e.data[0]==0xf5 && n++>=frames) {++out.f5;out.hold_rpm.push_back(((e.data[1]&0x0fu)<<8)|e.data[2]);}
  }
  out.sync=out.sync>frames ? out.sync-frames : 0;return out;
}
}
TEST(ProgramGains, HoldStreamCanRepeatF5LessOftenWithoutStoppingSync) {
  const auto every=hold_run(1,0.),sparse=hold_run(5,0.);
  ASSERT_GT(every.sync,20u);ASSERT_GT(sparse.sync,20u);
  EXPECT_NEAR(every.f5,every.sync,2);
  EXPECT_NEAR(sparse.f5,sparse.sync/5.,3);
}
TEST(ProgramGains, HoldSpeedOverrideSetsFloorOfHoldFrames) {
  const auto standard=hold_run(1,0.),slow=hold_run(1,2.);
  ASSERT_FALSE(standard.hold_rpm.empty());ASSERT_FALSE(slow.hold_rpm.empty());
  EXPECT_LT(*std::min_element(slow.hold_rpm.begin(),slow.hold_rpm.end()),*std::min_element(standard.hold_rpm.begin(),standard.hold_rpm.end()));
}
TEST(ProgramGains, HoldDiagnosticsAreValidated) {
  ExecutionTuning t{.025,1.,4.,2.,.01,-1,0.,false,1,0.};EXPECT_TRUE(validate_execution_tuning(t).empty());
  t.hold_period_cycles=0;EXPECT_FALSE(validate_execution_tuning(t).empty());
  t.hold_period_cycles=101;EXPECT_FALSE(validate_execution_tuning(t).empty());
  t.hold_period_cycles=1;t.hold_rpm=-1;EXPECT_FALSE(validate_execution_tuning(t).empty());
}
namespace {
struct FinishRun {std::string state;unsigned f7{},late{},collisions{},sync{},reads_after_last_f5{};std::vector<std::vector<std::uint8_t>> f5;bool fault{};};
FinishRun finish_run(ExecutionTuning t,unsigned frozen_axis=1) {
  Rig h(schedule_config(false));h.ready();h.synchronized();std::string reason;
  EXPECT_TRUE(h.bus->set_execution_tuning(t,reason))<<reason;
  if(frozen_axis) h.port->frozen[frozen_axis]=true;
  const auto before=h.port->events.size();h.execute(smooth(h,"PTP",.05,.2));h.run(1500);
  FinishRun out;const auto s=h.bus->snapshot();out.state=h.bus->program_status().state;out.fault=s.fault;out.late=static_cast<unsigned>(s.statistics.late);
  out.collisions=h.port->reply_collisions;
  for(auto i=before;i<h.port->events.size();++i) {
    const auto &e=h.port->events[i];
    if(e.axis==2 && e.data[0]==0xf7) ++out.f7;
    if(e.axis==2 && e.data[0]==0xf5) {out.f5.push_back(e.data);out.reads_after_last_f5=0;}
    if(e.axis==2 && e.data[0]==0x31) ++out.reads_after_last_f5;
    if(e.data[0]==0x4b) ++out.sync;
  }
  return out;
}
unsigned f5_units(const std::vector<std::uint8_t> &d) {return ((d[1]&0x0fu)<<8)|d[2];}
ExecutionTuning finish_tuning() {return {kBaseLookahead,1.,4.,2.,.01,-1,0.,false,1,0.};}
unsigned acc_zero_frames(const FinishRun &r) {return static_cast<unsigned>(std::count_if(r.f5.begin(),r.f5.end(),[](const auto &d){return d[3]==0;}));}
}
TEST(ProgramGains, FinishAccelerationAppliesOnlyToHoldFramesAndOptionalTail) {
  auto t=finish_tuning();const auto plain=finish_run(t);EXPECT_EQ(acc_zero_frames(plain),0u);
  t.hold_acceleration=0;const auto hold=finish_run(t);
  const unsigned trajectory=21;ASSERT_GT(hold.f5.size(),trajectory);
  for(unsigned i=0;i<trajectory;++i) EXPECT_NE(hold.f5[i][3],0)<<i;
  EXPECT_EQ(acc_zero_frames(hold),hold.f5.size()-trajectory);
  t.hold_tail_ms=100;const auto tail=finish_run(t);
  for(unsigned i=0;i<10;++i) EXPECT_NE(tail.f5[i][3],0)<<i;
  for(unsigned i=11;i<trajectory;++i) EXPECT_EQ(tail.f5[i][3],0)<<i;
}
TEST(ProgramGains, TailWindowLimitsAccelerationToWindowFrames) {
  auto t=finish_tuning();t.hold_acceleration=0;t.hold_tail_ms=100;t.hold_tail_end_ms=50;const auto r=finish_run(t);
  const unsigned trajectory=21;ASSERT_GT(r.f5.size(),trajectory);
  for(unsigned i=0;i<r.f5.size();++i) {
    if(i>=11 && i<=14) EXPECT_EQ(r.f5[i][3],0)<<i;
    else if(i<10 || i>16) EXPECT_NE(r.f5[i][3],0)<<i;
  }
}
TEST(ProgramGains, TailWindowMarkScalesOnlyWindowFrameSpeed) {
  auto t=finish_tuning();const auto plain=finish_run(t);
  t.hold_tail_ms=100;t.hold_tail_end_ms=50;t.hold_mark_scale=2;const auto marked=finish_run(t);
  ASSERT_GT(plain.f5.size(),21u);ASSERT_GT(marked.f5.size(),21u);
  for(unsigned i:{5u,8u,17u,19u}) EXPECT_EQ(f5_units(marked.f5[i]),f5_units(plain.f5[i]))<<i;
  for(unsigned i:{11u,12u,13u}) {
    ASSERT_GT(f5_units(plain.f5[i]),1u)<<i;EXPECT_EQ(f5_units(marked.f5[i]),2*f5_units(plain.f5[i]))<<i;
    unsigned sum=2;for(unsigned b=0;b<7;++b) sum+=marked.f5[i][b];EXPECT_EQ(marked.f5[i][7],static_cast<std::uint8_t>(sum))<<i;
  }
}
TEST(ProgramGains, SilentHoldSendsOneTargetFrameAndKeepsReadingPositions) {
  const auto stream=finish_run(finish_tuning(),0);
  auto t=finish_tuning();t.hold_silent=1;const auto r=finish_run(t,0);
  EXPECT_EQ(r.state,"succeeded");EXPECT_FALSE(r.fault);EXPECT_EQ(r.collisions,0u);
  EXPECT_EQ(r.f5.size(),22u);EXPECT_EQ(r.sync,22u);EXPECT_GT(stream.f5.size(),r.f5.size());
  EXPECT_GE(r.reads_after_last_f5,10u);
}
TEST(ProgramGains, SilentHoldStillCorrectsAxisOutsideTolerance) {
  const auto stream=finish_run(finish_tuning());
  auto t=finish_tuning();t.hold_silent=1;const auto r=finish_run(t);
  EXPECT_FALSE(r.fault);EXPECT_EQ(r.collisions,0u);
  EXPECT_GT(r.f5.size(),22u);EXPECT_LT(r.f5.size(),stream.f5.size());EXPECT_EQ(r.state,stream.state);
}
TEST(ProgramGains, StopAtTargetSendsF7OnceAndEndsHoldStreamForThatAxis) {
  auto t=finish_tuning();t.hold_stop_mode=1;t.hold_stop_ticks=150;const auto r=finish_run(t);
  EXPECT_EQ(r.f7,1u);EXPECT_LE(r.f5.size(),24u);EXPECT_FALSE(r.fault);EXPECT_EQ(r.late,0u);EXPECT_EQ(r.collisions,0u);
}
TEST(ProgramGains, StopAtTargetCanUseF5StopThroughSync) {
  auto t=finish_tuning();t.hold_stop_mode=2;t.hold_stop_ticks=150;const auto r=finish_run(t);
  EXPECT_EQ(r.f7,0u);ASSERT_FALSE(r.f5.empty());EXPECT_FALSE(r.fault);
  const auto &last=r.f5.back();EXPECT_EQ(last[1],0);EXPECT_EQ(last[2],0);EXPECT_EQ(last[3],0);EXPECT_EQ(last[4],0);EXPECT_EQ(last[5],0);EXPECT_EQ(last[6],0);
}
TEST(ProgramGains, StopAtTargetCompletesProgramWhenAllAxesArrive) {
  for(unsigned mode:{1u,2u}) {auto t=finish_tuning();t.hold_stop_mode=mode;t.hold_stop_ticks=150;const auto r=finish_run(t,0);EXPECT_EQ(r.state,"succeeded")<<mode;EXPECT_FALSE(r.fault);}
}
TEST(ProgramGains, FinishDiagnosticsAreValidated) {
  auto t=finish_tuning();t.hold_acceleration=256;EXPECT_FALSE(validate_execution_tuning(t).empty());
  t=finish_tuning();t.hold_tail_ms=2001;EXPECT_FALSE(validate_execution_tuning(t).empty());
  t=finish_tuning();t.hold_stop_mode=3;t.hold_stop_ticks=10;EXPECT_FALSE(validate_execution_tuning(t).empty());
  t=finish_tuning();t.hold_stop_mode=1;EXPECT_FALSE(validate_execution_tuning(t).empty());
  t=finish_tuning();t.hold_tail_end_ms=2001;EXPECT_FALSE(validate_execution_tuning(t).empty());
  t=finish_tuning();t.hold_mark_scale=.2;EXPECT_FALSE(validate_execution_tuning(t).empty());
  t=finish_tuning();t.hold_mark_scale=4.5;EXPECT_FALSE(validate_execution_tuning(t).empty());
  t=finish_tuning();t.hold_silent=2;EXPECT_FALSE(validate_execution_tuning(t).empty());
  t=finish_tuning();t.hold_tail_end_ms=500;t.hold_tail_ms=0;EXPECT_TRUE(validate_execution_tuning(t).empty());   // kolejność zapisu dowolna
}
TEST(ProgramGains, StopAtTargetDoesNotQueryAxisBeforeItsF7Reply) {
  // Zapytanie wysłane tuż po F7 zderzałoby się z odpowiedzią F7 o tym samym identyfikatorze CAN.
  Rig h(schedule_config(false));h.ready();h.synchronized();std::string reason;auto t=finish_tuning();t.hold_stop_mode=1;t.hold_stop_ticks=150;
  ASSERT_TRUE(h.bus->set_execution_tuning(t,reason))<<reason;h.execute(smooth(h,"PTP",.05,.2));h.run(1500);
  EXPECT_EQ(h.bus->program_status().state,"succeeded")<<h.bus->program_status().message;
  for(unsigned axis=1;axis<=6;++axis) {
    Time f7{};bool seen=false;
    for(const auto &e:h.port->events) if(e.axis==axis) {
      if(e.data[0]==0xf7 && !seen) {f7=e.stamp;seen=true;}
      else if(seen && e.stamp==f7) ADD_FAILURE()<<"J"<<axis<<": ramka 0x"<<std::hex<<unsigned(e.data[0])<<" w tym samym takcie co F7";
    }
    EXPECT_TRUE(seen)<<axis;
  }
}

