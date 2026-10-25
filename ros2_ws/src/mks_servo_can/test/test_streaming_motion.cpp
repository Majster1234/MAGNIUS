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
  EXPECT_NEAR(moving.lookahead_position,.285,1e-12);EXPECT_EQ(moving.rpm,9);EXPECT_EQ(moving.bytes[3],135);
  const auto holding=streaming_setpoint(a,.2,0);EXPECT_NEAR(holding.lookahead_position,.2,1e-12);EXPECT_EQ(holding.rpm,1);
  EXPECT_EQ(holding.bytes[0],0xf5);EXPECT_NE(holding.rpm,0);
  a.streaming.max_rpm=8;EXPECT_EQ(streaming_setpoint(a,.2,1).rpm,8);
}
TEST(StreamingMath, GearDirectionOffsetAndMicrostepAreApplied) {
  auto a=axis();a.gear_ratio=10;a.direction=-1;a.offset=.1;
  const auto s=streaming_setpoint(a,.3,1);
  EXPECT_LT(s.ticks,0);EXPECT_NEAR(s.ticks,std::llround(-.285*10*16384/(2*pi)),1);
  a.streaming.min_holding_rpm=3;
  EXPECT_EQ(streaming_setpoint(a,.3,0,128).rpm,24); // raw ×0,125 RPM
}
TEST(StreamingMath, DeltaSigmaTracksFractionalRpmOnIndependentAxes) {
  auto a=axis();std::array<StreamingState,2> states{};
  for(unsigned j=0;j<2;++j) {
    const double desired=j ? 2.78 : 1.73;unsigned total=0,upper=0;
    for(unsigned i=0;i<100;++i) {
      const auto s=streaming_setpoint(a,0,desired*2*pi/60,states[j]);
      EXPECT_GE(s.rpm,j ? 2u : 1u);EXPECT_LE(s.rpm,j ? 3u : 2u);
      total+=s.rpm;upper+=s.rpm==(j ? 3 : 2);
    }
    EXPECT_NEAR(total/100.,desired,.02);EXPECT_EQ(upper,j ? 78u : 73u);
  }
}
TEST(StreamingMath, SignedTrackingCorrectionSlowsLeadingAxisAndSpeedsLaggingAxis) {
  auto a=axis();const double v=4.73*2*pi/60;
  for(int direction:{-1,1}) {
    double leading=0,lagging=0;StreamingState lead,lag;
    for(unsigned i=0;i<100;++i) {
      leading+=streaming_setpoint(a,0,direction*v,lead,direction*.02).rpm;
      lagging+=streaming_setpoint(a,0,direction*v,lag,-direction*.02).rpm;
    }
    EXPECT_LT(leading/100.,4.73);EXPECT_GT(lagging/100.,4.73);
    EXPECT_NEAR(leading/100.,4.73-.05*60/(2*pi),.02);
    EXPECT_NEAR(lagging/100.,4.73+.05*60/(2*pi),.02);
  }
}
TEST(StreamingMath, DitherResetsAtDirectionChangeAndHonorsHoldingAndLimits) {
  auto a=axis();StreamingState state;const double v=1.73*2*pi/60;
  EXPECT_EQ(streaming_setpoint(a,0,v,state).rpm,1);
  EXPECT_EQ(streaming_setpoint(a,0,-v,state).rpm,1);
  EXPECT_EQ(streaming_setpoint(a,0,0,state,std::nullopt).rpm,1);EXPECT_DOUBLE_EQ(state.rpm_dither_accum,0);
  EXPECT_EQ(streaming_setpoint(a,0,.01,state).rpm,1);
  a.streaming.max_rpm=2;for(unsigned i=0;i<100;++i) EXPECT_EQ(streaming_setpoint(a,0,1,state,-1.).rpm,2);
  a.streaming.position_kp=-1;EXPECT_THROW(streaming_setpoint(a,0,v,state),std::invalid_argument);
  a.streaming.position_kp=NAN;EXPECT_THROW(streaming_setpoint(a,0,v,state),std::invalid_argument);
}
TEST(StreamingMath, DitherWorksInMicrostepScaledUnitsWithoutStaleFeedbackCorrection) {
  auto a=axis();StreamingState state;double total=0;
  for(unsigned i=0;i<100;++i) total+=streaming_setpoint(a,0,1.73*2*pi/60,state,std::nullopt,128).rpm*.125;
  EXPECT_NEAR(total/100.,1.73,.002);
}
TEST(StreamingMath, PhysicalLimitsAndSignedPositionClamp) {
  auto a=axis();a.upper=.3;const auto s=streaming_setpoint(a,.29,1);EXPECT_DOUBLE_EQ(s.lookahead_position,.3);
  EXPECT_THROW(streaming_setpoint(a,.4,1),std::invalid_argument);
  a=axis();a.gear_ratio=1e10;EXPECT_EQ(streaming_setpoint(a,9,0).ticks,8388607);
  EXPECT_EQ(streaming_setpoint(a,-9,0).ticks,-8388608);
  a=axis();a.max_velocity=.1;EXPECT_THROW(streaming_setpoint(a,0,0),std::invalid_argument);
  a=axis();a.streaming.encoder_cpr=4096;EXPECT_THROW(streaming_setpoint(a,0,0),std::invalid_argument);
}
TEST(StreamingAccuracy, BrakingDoesNotOvershootTarget) {
  auto a=axis();StreamingState state;
  const auto sample=streaming_setpoint(a,.33,1.2,state,std::nullopt,16,-18.,.025);
  const double stop=.33+1.2*.025-.5*18*.025*.025;
  EXPECT_NEAR(sample.lookahead_position,stop,1e-12);
  EXPECT_LE(sample.lookahead_position,stop+1e-12);
  const auto clipped=streaming_setpoint(a,.33,1.2,state,std::nullopt,16,-18.,.2);
  EXPECT_NEAR(clipped.lookahead_position,.33+1.2*1.2/(2*18),1e-12);
  EXPECT_NEAR(streaming_setpoint(a,-.33,-1.2,state,std::nullopt,16,18.,.025).lookahead_position,-stop,1e-12);
  EXPECT_DOUBLE_EQ(streaming_setpoint(a,.35,0,state,std::nullopt,16,18.,0).lookahead_position,.35);
}
TEST(StreamingAccuracy, LatencyCompensationZeroesPhantomLag) {
  auto a=axis();a.gear_ratio=10;StreamingState delayed,fresh;unsigned sum=0;
  for(unsigned i=0;i<100;++i) {
    const auto sample=streaming_setpoint(a,.5,1.,delayed,.485,16,0.,1.,.015);
    EXPECT_EQ(sample.rpm,streaming_setpoint(a,.5,1.,fresh,.5,16,0.,1.,0.).rpm);
    sum+=sample.rpm;
  }
  EXPECT_NEAR(sum/100.,60*10/(2*pi),.02);
  for(int sign:{-1,1}) {
    StreamingState old,current;
    const double q=sign*.5,v=sign*.7,acc=sign*2.,age=.015;
    const double actual=q-v*age+.5*acc*age*age;
    EXPECT_EQ(streaming_setpoint(a,q,v,old,actual,16,acc,1,age).rpm,
              streaming_setpoint(a,q,v,current,q,16,acc,1,0).rpm);
  }
}
TEST(StreamingAccuracy, EndpointPositionErrorDrivesCorrectionWhenVelocityIsZero) {
  auto a=axis();a.gear_ratio=10;
  for(double actual:{.33,.37}) {
    StreamingState state;
    const auto sample=streaming_setpoint(a,.35,0.,state,actual,16,0.,0.);
    EXPECT_GT(sample.rpm,a.streaming.min_holding_rpm);
    EXPECT_EQ(state.direction,actual<.35 ? 1 : -1);
    EXPECT_DOUBLE_EQ(sample.lookahead_position,.35);
    EXPECT_NEAR(sample.rpm,2.5*.02*10*60/(2*pi),1.);
  }
  StreamingState state;
  EXPECT_EQ(streaming_setpoint(a,.35,0.,state,.3502,16,0.,0.).rpm,1u);
  EXPECT_EQ(state.direction,0);
  EXPECT_EQ(streaming_setpoint(a,.35,0.,state,std::nullopt,16,0.,0.).rpm,1u);
}
TEST(StreamingAccuracy, ValidationLegacyWrappersAndAgeClamp) {
  auto a=axis();StreamingState state;
  for(double bad:std::array<double,3>{-1.,NAN,INFINITY}) {
    EXPECT_THROW(streaming_setpoint(a,0,1,state,std::nullopt,16,0,bad,0),std::invalid_argument);
    EXPECT_THROW(streaming_setpoint(a,0,1,state,std::nullopt,16,0,1,bad),std::invalid_argument);
  }
  EXPECT_THROW(streaming_setpoint(a,0,1,state,std::nullopt,16,NAN,1,0),std::invalid_argument);
  EXPECT_THROW(streaming_setpoint(a,0,1,state,std::nullopt,16,INFINITY,1,0),std::invalid_argument);
  StreamingState legacy,explicit_call;
  EXPECT_EQ(streaming_setpoint(a,.2,.7,legacy,.18,32).bytes,
            streaming_setpoint(a,.2,.7,explicit_call,.18,32,0,a.streaming.lookahead_time_sec,0).bytes);
  StreamingState clamped,limit;
  EXPECT_EQ(streaming_setpoint(a,.5,1,clamped,.45,16,0,1,1).bytes,
            streaming_setpoint(a,.5,1,limit,.45,16,0,1,.05).bytes);
  EXPECT_NEAR(streaming_setpoint(a,0,1).lookahead_position,.085,1e-12);
  EXPECT_NEAR(streaming_bus_utilization(6,1000000),.2658,1e-12);
}
TEST(RealtimeMailbox, LatestCommandIsCoherentAndNeverReplaysAnOldVersion) {
  CommandMailbox box;std::atomic<bool> done{false};
  std::thread writer([&]{for(unsigned n=1;n<=20000;++n) {CommandBatch b;b.sequence=n;b.configuration_generation=n;b.positions.fill(n);b.velocities.fill(2*n);b.velocities_valid=true;box.publish(b);}done=true;});
  std::uint64_t seen=0,last=0;CommandBatch b;
  while(!done || last<20000) if(box.take(b,seen)) {
    EXPECT_GE(b.sequence,last);last=b.sequence;EXPECT_EQ(b.configuration_generation,b.sequence);
    for(unsigned j=0;j<6;++j) {EXPECT_EQ(b.positions[j],static_cast<double>(b.sequence));EXPECT_EQ(b.velocities[j],2.*b.sequence);}
  }
  writer.join();EXPECT_FALSE(box.take(b,seen));
}
TEST(RealtimeMailbox, ControlStateKeepsMatchingPositionAndTimestamp) {
  ControlMailbox box;std::atomic<bool> done{false};
  std::thread writer([&]{for(unsigned n=1;n<=10000;++n) {ControlState s;s.active=true;s.configuration_ready=true;s.configuration_generation=n;for(unsigned j=0;j<6;++j) s.conversions[j]={double(n),-1,-double(n),static_cast<uint16_t>(j+1),true};for(auto &p:s.positions) p={static_cast<double>(n),Time(std::chrono::nanoseconds(n)),true};box.publish(s);}done=true;});
  ControlState state;
  while(!done) if(box.read(state) && state.positions[0].valid) for(const auto &p:state.positions) {
    EXPECT_TRUE(state.configuration_ready);EXPECT_EQ(state.configuration_generation,p.value);EXPECT_EQ(state.conversions[0].gear_ratio,p.value);EXPECT_EQ(state.conversions[0].offset,-p.value);
    EXPECT_EQ(p.value,state.positions[0].value);EXPECT_EQ(p.value,std::chrono::duration_cast<std::chrono::nanoseconds>(p.stamp.time_since_epoch()).count());
  }
  writer.join();
}
