#include <gtest/gtest.h>
#include "mks_servo_can/streaming_motion.hpp"
#include "mks_servo_can/realtime_mailbox.hpp"
#include <cmath>
#include <thread>
using namespace mks_servo_can;
namespace {
constexpr double pi=3.14159265358979323846;
AxisConfig axis() {return {1,1,1,0,-10,10,10,100,.1,240};}
}
TEST(StreamingCodec, PositiveNegativeAndSigned24BoundaryBytesHaveChecksums) {
  EXPECT_EQ(absolute_position_request(1,16384,600,2),(std::vector<uint8_t>{0xf5,2,0x58,2,0,0x40,0,0x92}));
  EXPECT_EQ(absolute_position_request(1,-16384,600,2),(std::vector<uint8_t>{0xf5,2,0x58,2,0xff,0xc0,0,0x11}));
  const auto minimum=absolute_position_request(6,-8388608,2500,240);
  EXPECT_EQ(minimum[4],0x80);EXPECT_EQ(minimum[5],0);EXPECT_EQ(minimum[6],0);
  EXPECT_TRUE(validate_response(6,minimum,0xf5,8).valid);
  EXPECT_EQ(encoder_position_request(5),(std::vector<uint8_t>{0x31,0x36}));
}
TEST(StreamingCodec, Signed48ExtremesAndInvalidFrames) {
  for(std::int64_t value:{std::int64_t{0},std::int64_t{16384},std::int64_t{-16384},(std::int64_t{1}<<47)-1,-(std::int64_t{1}<<47)}) {
    std::vector<uint8_t> bytes{0x31};const auto raw=static_cast<uint64_t>(value);
    for(int shift=40;shift>=0;shift-=8) bytes.push_back(raw>>shift);
    bytes.push_back(checksum(3,bytes));ASSERT_TRUE(parse_position(3,bytes));EXPECT_EQ(*parse_position(3,bytes),value);
    EXPECT_FALSE(parse_position(4,bytes));bytes.back()^=1;EXPECT_FALSE(parse_position(3,bytes));bytes.pop_back();EXPECT_FALSE(parse_position(3,bytes));
  }
}
TEST(StreamingMath, LookaheadFeedForwardAndHoldingSpeed) {
  auto a=axis();const auto moving=streaming_setpoint(a,.2,1);
  EXPECT_NEAR(moving.lookahead_position,.225,1e-12);EXPECT_EQ(moving.rpm,11);EXPECT_EQ(moving.bytes[3],240);
  const auto holding=streaming_setpoint(a,.2,0);EXPECT_NEAR(holding.lookahead_position,.2,1e-12);EXPECT_EQ(holding.rpm,3);
  EXPECT_EQ(holding.bytes[0],0xf5);EXPECT_NE(holding.rpm,0);
  a.streaming.max_rpm=10;EXPECT_EQ(streaming_setpoint(a,.2,1).rpm,10);
}
TEST(StreamingMath, GearDirectionOffsetAndMicrostepAreApplied) {
  auto a=axis();a.gear_ratio=10;a.direction=-1;a.offset=.1;
  const auto s=streaming_setpoint(a,.3,1);
  EXPECT_LT(s.ticks,0);EXPECT_NEAR(s.ticks,std::llround(-.225*10*16384/(2*pi)),1);
  a.streaming.min_holding_rpm=3;
  EXPECT_EQ(streaming_setpoint(a,.3,0,128).rpm,24); // raw ×0,125 RPM
}
TEST(StreamingMath, PhysicalLimitsAndSignedPositionClamp) {
  auto a=axis();a.upper=.3;const auto s=streaming_setpoint(a,.29,1);EXPECT_DOUBLE_EQ(s.lookahead_position,.3);
  EXPECT_THROW(streaming_setpoint(a,.4,1),std::invalid_argument);
  a=axis();a.gear_ratio=1e10;EXPECT_EQ(streaming_setpoint(a,9,0).ticks,8388607);
  EXPECT_EQ(streaming_setpoint(a,-9,0).ticks,-8388608);
  a=axis();a.max_velocity=.1;EXPECT_THROW(streaming_setpoint(a,0,0),std::invalid_argument);
  a=axis();a.streaming.encoder_cpr=4096;EXPECT_THROW(streaming_setpoint(a,0,0),std::invalid_argument);
}
TEST(RealtimeMailbox, LatestCommandIsCoherentAndNeverReplaysAnOldVersion) {
  CommandMailbox box;std::atomic<bool> done{false};
  std::thread writer([&]{for(unsigned n=1;n<=20000;++n) {CommandBatch b;b.sequence=n;b.positions.fill(n);b.velocities.fill(2*n);b.velocities_valid=true;box.publish(b);}done=true;});
  std::uint64_t seen=0,last=0;CommandBatch b;
  while(!done || last<20000) if(box.take(b,seen)) {
    EXPECT_GE(b.sequence,last);last=b.sequence;
    for(unsigned j=0;j<6;++j) {EXPECT_EQ(b.positions[j],static_cast<double>(b.sequence));EXPECT_EQ(b.velocities[j],2.*b.sequence);}
  }
  writer.join();EXPECT_FALSE(box.take(b,seen));
}
TEST(RealtimeMailbox, ControlStateKeepsMatchingPositionAndTimestamp) {
  ControlMailbox box;std::atomic<bool> done{false};
  std::thread writer([&]{for(unsigned n=1;n<=10000;++n) {ControlState s;s.active=true;for(auto &p:s.positions) p={static_cast<double>(n),Time(std::chrono::nanoseconds(n)),true};box.publish(s);}done=true;});
  ControlState state;
  while(!done) if(box.read(state) && state.positions[0].valid) for(const auto &p:state.positions) {
    EXPECT_EQ(p.value,state.positions[0].value);EXPECT_EQ(p.value,std::chrono::duration_cast<std::chrono::nanoseconds>(p.stamp.time_since_epoch()).count());
  }
  writer.join();
}
