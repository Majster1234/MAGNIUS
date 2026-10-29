#include <gtest/gtest.h>
#include <cmath>
#include "streaming_fixture.hpp"
using namespace streaming_test;
namespace {
BusConfig tcp_config() {auto c=config();c.tcp_jog_enabled=true;return c;}
std::uint64_t begin(Rig &rig,std::uint64_t sequence=1) {
  rig.bus->update_motion_model("model");return rig.bus->begin_tcp_jog("session","bench","model",sequence,rig.now,rig.now);
}
CommandBatch batch(Rig &rig,std::uint64_t generation,std::uint64_t sequence) {
  CommandBatch b;b.source=Source::tcp_jog;b.control_generation=generation;b.configuration_generation=rig.bus->control_state().configuration_generation;
  b.sequence=sequence;b.created=rig.now;b.expires=rig.now+20ms;b.velocities_valid=true;return b;
}
}
TEST(TcpJogOwner, DisabledMissingAxisCalibrationAndWrongModelRejectStart) {
  Rig disabled;disabled.ready();EXPECT_EQ(begin(disabled),0u);
  Rig missing(tcp_config());missing.port->absent=6;missing.ready();EXPECT_EQ(begin(missing),0u);
  Rig rig(tcp_config());rig.ready();rig.bus->update_motion_model("model");
  EXPECT_EQ(rig.bus->begin_tcp_jog("session","other","model",1,rig.now,rig.now),0u);
  EXPECT_EQ(rig.bus->begin_tcp_jog("session","bench","other",1,rig.now,rig.now),0u);
  EXPECT_EQ(rig.bus->begin_tcp_jog("session","bench","model",1,rig.now-100ms,rig.now),0u);
}
TEST(TcpJogOwner, WatchdogStopsEvenWhenGeneratorKeepsPublishing) {
  Rig rig(tcp_config());rig.ready();const auto generation=begin(rig);ASSERT_NE(generation,0u);
  for(unsigned i=0;i<20;++i) {ASSERT_TRUE(rig.bus->submit(batch(rig,generation,i+1)));rig.run(10);}
  rig.run(2);EXPECT_TRUE(rig.bus->control_state().fault);const auto sent=rig.port->counts[0xf5];
  EXPECT_GT(rig.port->counts[0xf7],0u);EXPECT_FALSE(rig.bus->submit(batch(rig,generation,30)));
  const auto first=rig.port->events.size();rig.run(50);
  EXPECT_EQ(rig.port->counts[0xf5],sent+6);
  for(auto i=first;i<rig.port->events.size();++i) {
    const auto &e=rig.port->events[i];
    if(e.data[0]==0xf5) EXPECT_EQ(e.data,absolute_position_stop_request(e.axis));
  }
}
TEST(TcpJogOwner, SessionGenerationArbitrationAndReferenceTimeout) {
  Rig rig(tcp_config());rig.ready();const auto generation=begin(rig);ASSERT_NE(generation,0u);
  EXPECT_FALSE(rig.bus->activate(Source::trajectory,rig.now));
  rig.bus->renew_source_lease(Source::trajectory,rig.now);
  EXPECT_EQ(rig.bus->control_state().source,Source::tcp_jog);
  EXPECT_FALSE(rig.bus->tcp_jog_control("other",generation,2,rig.now,rig.now,false));
  EXPECT_FALSE(rig.bus->tcp_jog_control("session",generation+1,2,rig.now,rig.now,false));
  EXPECT_TRUE(rig.bus->tcp_jog_control("session",generation,2,rig.now,rig.now,false));
  EXPECT_FALSE(rig.bus->tcp_jog_control("session",generation,2,rig.now,rig.now,false));
  EXPECT_FALSE(rig.bus->submit(batch(rig,generation+1,1)));
  ASSERT_TRUE(rig.bus->submit(batch(rig,generation,1)));rig.run(12);
  ASSERT_TRUE(rig.bus->brake_tcp_jog("session",generation));
  ASSERT_TRUE(rig.bus->finish_tcp_jog("session",generation));
  const auto next=begin(rig,3);ASSERT_GT(next,generation);
  EXPECT_FALSE(rig.bus->submit(batch(rig,generation,100)));
  rig.run(65);EXPECT_TRUE(rig.bus->control_state().fault);
}
TEST(TcpJogOwner, MeasuredMechanicalLimitAndModelChangeStopBeforeNewCommands) {
  for(bool model_change:{false,true}) {
    Rig rig(tcp_config());rig.ready();const auto generation=begin(rig);ASSERT_NE(generation,0u);
    if(model_change) rig.bus->update_motion_model("different");
    else {rig.port->frozen[1]=true;rig.port->positions[1]=static_cast<std::int64_t>(2.2*10*16384/(2*std::acos(-1.)));}
    for(unsigned i=0;i<5 && !rig.bus->control_state().fault;++i) {
      ASSERT_TRUE(rig.bus->submit(batch(rig,generation,i+1)));rig.run(10);
    }
    EXPECT_TRUE(rig.bus->control_state().fault);
  }
}
