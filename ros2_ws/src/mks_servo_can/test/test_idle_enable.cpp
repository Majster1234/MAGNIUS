#include <gtest/gtest.h>
#include "mks_servo_can/idle_enable_monitor.hpp"
#include "streaming_fixture.hpp"

using namespace mks_servo_can;
using namespace std::chrono_literals;
namespace {
using Positions=std::array<std::int64_t,kAxes>;
bool advance(IdleEnableMonitor &monitor,Time &now,Clock::duration elapsed,const Positions &positions,unsigned mask=63) {
  const auto end=now+elapsed;bool due=false;
  while(now<end) {now=std::min(end,now+100ms);due=monitor.observe(now,positions,true,mask);}
  return due;
}
void advance_bus(streaming_test::Rig &rig,Clock::duration elapsed) {
  const auto end=rig.now+elapsed;unsigned steps=0;
  while(rig.now<end) {
    rig.port->now=rig.now;rig.bus->tick(rig.now);rig.now+=10ms;
    if(++steps%1000==0) rig.port->events.clear();
  }
}
}
TEST(IdleEnable, DisablesOnlyAfterAFullHourAndIgnoresEncoderNoiseBelowTwoDegrees) {
  IdleEnableMonitor monitor;Time now{1s};Positions positions{};
  EXPECT_FALSE(monitor.observe(now,positions,true,63));
  positions[0]=91; // 1,9995° wału, poniżej progu 2°.
  EXPECT_FALSE(advance(monitor,now,60min-100ms,positions));
  EXPECT_TRUE(advance(monitor,now,100ms,positions));
}
TEST(IdleEnable, MoreThanTwoDegreesOnAnyMotorRestartsTheWholeTimer) {
  for(unsigned j=0;j<kAxes;++j) {
    IdleEnableMonitor monitor;Time now{1s};Positions positions{};
    monitor.observe(now,positions,true,63);
    EXPECT_FALSE(advance(monitor,now,59min,positions));
    positions[j]=92; // Pierwsza reprezentowalna pozycja ponad 2° wału.
    EXPECT_FALSE(advance(monitor,now,100ms,positions));
    EXPECT_FALSE(advance(monitor,now,60min-100ms,positions));
    EXPECT_TRUE(advance(monitor,now,100ms,positions));
  }
}
TEST(IdleEnable, ReturnMotionCrossingTwoDegreesAlsoRestartsTheTimer) {
  IdleEnableMonitor monitor;Time now{1s};Positions positions{};
  monitor.observe(now,positions,true,63);
  positions[2]=50;advance(monitor,now,100ms,positions);
  positions[2]=-50;advance(monitor,now,100ms,positions);
  EXPECT_FALSE(advance(monitor,now,60min-100ms,positions));
  EXPECT_TRUE(advance(monitor,now,100ms,positions));
}
TEST(IdleEnable, MissingFeedbackBusyStateAndClockGapCannotCountAsInactivity) {
  for(unsigned reason=0;reason<4;++reason) {
    IdleEnableMonitor monitor;Time now{1s};Positions positions{};
    monitor.observe(now,positions,true,63);advance(monitor,now,59min,positions);
    if(reason==0) monitor.observe(now,positions,false,63);
    else if(reason==1) monitor.observe(now,positions,true,0);
    else if(reason==2) now+=1h;
    else now-=1h;
    EXPECT_FALSE(monitor.observe(now,positions,true,63));
    EXPECT_FALSE(advance(monitor,now,60min-100ms,positions));
    EXPECT_TRUE(advance(monitor,now,100ms,positions));
  }
}
TEST(IdleEnable, EnableChangeStartsANewHour) {
  IdleEnableMonitor monitor;Time now{1s};Positions positions{};
  monitor.observe(now,positions,true,1);advance(monitor,now,59min,positions,1);
  EXPECT_FALSE(monitor.observe(now,positions,true,63));
  EXPECT_FALSE(advance(monitor,now,60min-100ms,positions));
  EXPECT_TRUE(advance(monitor,now,100ms,positions));
}
TEST(IdleEnable, OwnerConfirmsDisableOfAllMotorsAndRequiresExplicitReenable) {
  streaming_test::Rig rig;rig.ready();
  advance_bus(rig,60min-2s);
  ASSERT_EQ(rig.port->counts[0xf3],0u);
  advance_bus(rig,8s);
  ASSERT_FALSE(rig.bus->snapshot().fault)<<rig.bus->snapshot().message;
  ASSERT_FALSE(rig.bus->manual().busy());
  for(unsigned j=1;j<=6;++j) EXPECT_FALSE(rig.port->enabled[j]);
  EXPECT_EQ(rig.port->counts[0xf3],6u);
  const auto state=rig.bus->manual().snapshot(rig.now);
  EXPECT_NE(state.message.find("Automatyczne wyłączenie ENABLE"),std::string::npos);
  EXPECT_NE(state.message.find("Wyłączenie potwierdzone"),std::string::npos);
  advance_bus(rig,10s);EXPECT_EQ(rig.port->counts[0xf3],6u);
  ASSERT_TRUE(rig.bus->manual().set_enable_all(true,rig.now));
  advance_bus(rig,5s);
  ASSERT_FALSE(rig.bus->snapshot().fault)<<rig.bus->snapshot().message;
  for(unsigned j=1;j<=6;++j) EXPECT_TRUE(rig.port->enabled[j]);
  EXPECT_EQ(rig.bus->manual().snapshot(rig.now).message.find("Automatyczne wyłączenie ENABLE"),std::string::npos);
}
