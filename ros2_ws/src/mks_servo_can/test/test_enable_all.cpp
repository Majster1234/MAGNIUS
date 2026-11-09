#include <gtest/gtest.h>
#include "streaming_fixture.hpp"
using namespace streaming_test;
TEST(EnableAll, PreparesAxesThenEnablesInOrderAtLeast200MsApartWithMeasuredConfirmation) {
  Rig rig;rig.ready();ASSERT_TRUE(rig.bus->manual().set_enable_all(true,rig.now));
  EXPECT_TRUE(rig.bus->manual().busy());EXPECT_EQ(rig.bus->manual().snapshot(rig.now).axis,0u);
  EXPECT_FALSE(rig.bus->manual().set_enable(1,false,rig.now));
  EXPECT_FALSE(rig.bus->activate(Source::trajectory,rig.now));
  rig.run(2500);ASSERT_FALSE(rig.bus->snapshot().fault)<<rig.bus->snapshot().message;
  EXPECT_FALSE(rig.bus->manual().busy());std::vector<Time> stamps;
  for(const auto &event:rig.port->events) if(event.data[0]==0xf3 && event.data[1]==1) stamps.push_back(event.stamp);
  ASSERT_EQ(stamps.size(),6u);for(unsigned j=1;j<6;++j) EXPECT_GE(stamps[j]-stamps[j-1],200ms);
  unsigned expected_axis=1;
  for(const auto &event:rig.port->events) if(event.data[0]==0xf3 && event.data[1]==1) EXPECT_EQ(event.axis,expected_axis++);
  for(unsigned axis=1;axis<=6;++axis) {EXPECT_TRUE(rig.port->enabled[axis]);EXPECT_TRUE(rig.bus->manual().prepared(axis));}
  EXPECT_EQ(rig.port->counts[0xf5],0u);
}
TEST(EnableAll, DisablesAvailableAxes200MsApartAndSkipsMissingAxis) {
  Rig rig;rig.port->absent=6;rig.ready();ASSERT_TRUE(rig.bus->manual().set_enable_all(false,rig.now));rig.run(2500);
  ASSERT_FALSE(rig.bus->snapshot().fault)<<rig.bus->snapshot().message;EXPECT_FALSE(rig.bus->manual().busy());
  std::vector<Time> stamps;
  for(const auto &event:rig.port->events) if(event.data[0]==0xf3 && event.data[1]==0) stamps.push_back(event.stamp);
  ASSERT_EQ(stamps.size(),5u);for(unsigned j=1;j<5;++j) EXPECT_GE(stamps[j]-stamps[j-1],200ms);
  for(unsigned axis=1;axis<=5;++axis) EXPECT_FALSE(rig.port->enabled[axis]);
}
TEST(EnableAll, StopOrMissingConfirmationAfterFirstEnableNeverEnablesNextAxis) {
  for(bool stop:{false,true}) {
    Rig rig;rig.ready();ASSERT_TRUE(rig.bus->manual().set_enable_all(true,rig.now));
    const auto enabled_count=[&] {return std::count_if(rig.port->events.begin(),rig.port->events.end(),[](const auto &e){return e.data[0]==0xf3 && e.data[1]==1;});};
    for(unsigned i=0;i<2000 && !enabled_count();++i) rig.run(1);
    ASSERT_EQ(enabled_count(),1);
    if(stop) rig.bus->stop("STOP podczas sekwencji ENABLE");
    else {rig.port->absent=1;rig.port->rx.clear();}
    rig.run(2500);EXPECT_NE(rig.bus->recovery_state_name(),"idle");EXPECT_EQ(enabled_count(),1);
    for(unsigned j=1;j<=6;++j) if(j!=rig.port->absent) EXPECT_FALSE(rig.port->enabled[j]);
  }
}
TEST(EnableAll, PreparationFailureAndStopNeverEnableRemainingAxes) {
  for(bool stop:{false,true}) {
    Rig rig;rig.ready();rig.port->bad_watchdog=!stop;ASSERT_TRUE(rig.bus->manual().set_enable_all(true,rig.now));
    if(stop) {rig.run(5);rig.bus->stop("STOP operatora");}
    rig.run(3000);EXPECT_NE(rig.bus->recovery_state_name(),"idle");
    for(const auto &event:rig.port->events) EXPECT_FALSE(event.data[0]==0xf3 && event.data[1]==1);
  }
}

TEST(EnableAll, CriticalFaultStopsImmediatelyAndDisablesAllAfterGuardGaps) {
  Rig rig;rig.ready();ASSERT_TRUE(rig.bus->manual().set_enable_all(true,rig.now));
  const auto count_enabled=[&] {return std::count_if(rig.port->events.begin(),rig.port->events.end(),[](const auto &e){return e.data[0]==0xf3 && e.data[1]==1;});};
  for(unsigned i=0;i<2000 && !count_enabled();++i) rig.run(1);
  ASSERT_EQ(count_enabled(),1);
  const auto begin=rig.port->events.size();const auto fault_stamp=rig.port->now;rig.bus->stop("krytyczna awaria podczas ENABLE");
  rig.run(13);
  std::vector<unsigned> disabled;
  for(auto i=begin;i<rig.port->events.size();++i) {
    const auto &e=rig.port->events[i];
    if(e.data[0]==0xf3 && e.data[1]==0) {disabled.push_back(e.axis);EXPECT_GE(e.stamp,fault_stamp+12ms);EXPECT_LE(e.stamp,fault_stamp+13ms);}
  }
  EXPECT_EQ(disabled,(std::vector<unsigned>{1,2,3,4,5,6}));
  rig.run(2000);EXPECT_EQ(count_enabled(),1);EXPECT_NE(rig.bus->recovery_state_name(),"idle");
  for(unsigned j=1;j<=6;++j) EXPECT_FALSE(rig.port->enabled[j]);
}

TEST(EnableAll, WaitsForShaftToSettleAfterPreparationInsteadOfFailing) {
  // 05.11 22:37: przygotowanie J1 (ENABLE=0 + F7) → wał osiadł o 6 impulsów, 97 ms później awaria „Zmiana pozycji”.
  Rig rig;rig.ready();ASSERT_TRUE(rig.bus->manual().set_enable_all(true,rig.now));rig.run(2500);
  ASSERT_FALSE(rig.bus->snapshot().fault)<<rig.bus->snapshot().message;
  rig.port->disable_settle[1]=6;
  ASSERT_TRUE(rig.bus->manual().set_enable(1,false,rig.now));rig.run(800);
  ASSERT_FALSE(rig.bus->manual().busy());ASSERT_FALSE(rig.port->enabled[1]);
  ASSERT_TRUE(rig.bus->manual().set_enable_all(true,rig.now));rig.run(3000);
  ASSERT_FALSE(rig.bus->snapshot().fault)<<rig.bus->snapshot().message;
  EXPECT_FALSE(rig.bus->manual().busy());
  for(unsigned axis=1;axis<=6;++axis) {EXPECT_TRUE(rig.port->enabled[axis]);EXPECT_TRUE(rig.bus->manual().prepared(axis));}
}
TEST(EnableAll, StillFailsWhenPositionNeverSettles) {
  Rig rig;rig.ready();ASSERT_TRUE(rig.bus->manual().set_enable_all(true,rig.now));rig.run(2500);
  ASSERT_FALSE(rig.bus->snapshot().fault)<<rig.bus->snapshot().message;
  ASSERT_TRUE(rig.bus->manual().set_enable(1,false,rig.now));rig.run(800);
  const auto stops=rig.port->counts[0xf7];
  ASSERT_TRUE(rig.bus->manual().set_enable_all(true,rig.now));
  // Wał J1 dryfuje od F7 w przygotowaniu (jak osiadanie po ENABLE=0, ale bez końca): po 1 s oczekiwania
  // przerwanie bez ENABLE J1.
  bool failed=false;
  for(unsigned i=0;i<4000 && !failed;++i) {
    if(rig.port->counts[0xf7]>stops) rig.port->positions[1]+=1;
    rig.run(1);failed=rig.bus->snapshot().fault;
  }
  EXPECT_TRUE(failed);
  EXPECT_NE(rig.bus->snapshot().message.find("Zmiana lub utrata pozycji przed ENABLE J1"),std::string::npos)<<rig.bus->snapshot().message;
  EXPECT_FALSE(rig.port->enabled[1]);
}
TEST(EnableAll, RepeatedEnableAllWhenEverythingIsPreparedDoesNotReportStaleTimeout) {
  // 05.11 22:56:31: „Załącz wszystkie” przy osiach już przygotowanych → fałszywe „Przekroczony czas operacji ręcznej”.
  Rig rig;rig.ready();ASSERT_TRUE(rig.bus->manual().set_enable_all(true,rig.now));rig.run(2500);
  ASSERT_FALSE(rig.bus->snapshot().fault)<<rig.bus->snapshot().message;
  rig.run(5000);  // termin poprzedniej operacji dawno minął
  ASSERT_TRUE(rig.bus->manual().set_enable_all(true,rig.now));rig.run(2500);
  ASSERT_FALSE(rig.bus->snapshot().fault)<<rig.bus->snapshot().message;
  EXPECT_FALSE(rig.bus->manual().busy());
  for(unsigned axis=1;axis<=6;++axis) EXPECT_TRUE(rig.port->enabled[axis]);
}
