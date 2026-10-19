#include <gtest/gtest.h>
#include <cmath>
#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/motion_sample.hpp"
using namespace mks_servo_can;
AxisConfig axis() { return {1,1,1,0,-10,10,10,100,.1,2}; }
TEST(MotionSample, PositionVelocityAndReferenceFrame) {
  auto a=axis(); const auto sample=compile_motor_sample(a,.02,1,0,.02);
  EXPECT_EQ(sample.ticks,52); EXPECT_EQ(sample.rpm,10); EXPECT_EQ(sample.acceleration,2);
  EXPECT_EQ(sample.payload,absolute_position_request(1,52,10,2)); EXPECT_EQ(sample.operation,1);
  a.gear_ratio=10;a.direction=-1;a.offset=.1;
  const auto reversed=compile_motor_sample(a,.3,1,.28,.02);
  EXPECT_LT(reversed.ticks,0); EXPECT_NEAR(CanBusManager::position_to_joint(a,reversed.ticks),.3,1e-4);
}
TEST(MotionSample, ZeroDerivativeIsNotStopAndLowSpeedIsReported) {
  const auto a=axis(); const auto hold=compile_motor_sample(a,.2,0,.2,.02);
  EXPECT_EQ(hold.rpm,1); EXPECT_EQ(hold.operation,0); EXPECT_EQ(hold.payload[0],0xf5);
  EXPECT_THROW(compile_motor_sample(a,.001,.05,0,.02),std::invalid_argument);
  EXPECT_NO_THROW(compile_motor_sample(a,.001,0,0,.02,false));
  EXPECT_THROW(compile_motor_sample(a,.2,NAN,.19,.02),std::invalid_argument);
  EXPECT_THROW(compile_motor_sample(a,.2,-1,.19,.02),std::invalid_argument);
}
TEST(MotionSample, RejectsUnknownAccPositionRangeAndVelocityLimit) {
  auto a=axis();a.acceleration_code=0; EXPECT_THROW(compile_motor_sample(a,0,0,0,.02),std::invalid_argument);
  a=axis();EXPECT_THROW(compile_motor_sample(a,11,0,0,.02),std::invalid_argument);
  EXPECT_THROW(compile_motor_sample(a,.1,11,0,.02),std::invalid_argument);
  a.gear_ratio=1e6; EXPECT_THROW(compile_motor_sample(a,9,0,9,.02),std::invalid_argument);
}
