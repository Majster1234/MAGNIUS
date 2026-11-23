#include <gtest/gtest.h>
#include <cmath>
#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/motion_sample.hpp"
using namespace mks_servo_can;
AxisConfig axis() {return {1,1,1,0,-10,10,10,100,.1,240};}
TEST(MotionSample, PreviewUsesSameLookaheadAndFeedForwardAsExecutor) {
  const auto a=axis();const auto sample=compile_motor_sample(a,.2,1,.19,.01);
  const auto actual=streaming_setpoint(a,.2,1);
  EXPECT_EQ(sample.ticks,actual.ticks);EXPECT_EQ(sample.rpm,9);EXPECT_EQ(sample.acceleration,135);
  EXPECT_EQ(sample.payload,std::vector<uint8_t>(actual.bytes.begin(),actual.bytes.end()));
}
TEST(MotionSample, ZeroDerivativeIsHoldingGoalAtMinimumRpm) {
  auto a=axis();const auto hold=compile_motor_sample(a,.2,0,.2,.01);
  EXPECT_EQ(hold.rpm,1);EXPECT_EQ(hold.operation,0);EXPECT_EQ(hold.payload[0],0xf5);
  EXPECT_NO_THROW(compile_motor_sample(a,.001,.05,0,.01));
  EXPECT_THROW(compile_motor_sample(a,.2,NAN,.19,.01),std::invalid_argument);
}
TEST(MotionSample, RejectsPositionVelocityAndPhysicalMinimumRpmViolations) {
  auto a=axis();EXPECT_THROW(compile_motor_sample(a,11,0,0,.01),std::invalid_argument);
  EXPECT_THROW(compile_motor_sample(a,.1,11,0,.01),std::invalid_argument);
  a.max_velocity=.1;EXPECT_THROW(compile_motor_sample(a,0,0,0,.01),std::invalid_argument);
}
