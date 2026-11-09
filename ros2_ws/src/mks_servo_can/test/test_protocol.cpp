#include <gtest/gtest.h>
#include "mks_servo_can/protocol.hpp"

using mks_servo_can::checksum;

TEST(Protocol, ParsesServo42Identity) {
  std::vector<std::uint8_t> frame{0x40, 0x12, 0x01, 0x00, 0x09, 0x20};
  frame.push_back(checksum(1, frame));
  const auto identity = mks_servo_can::parse_identity(1, frame);
  ASSERT_TRUE(identity.has_value());
  EXPECT_EQ(identity->model, mks_servo_can::Model::kServo42D);
  EXPECT_EQ(identity->firmware_minor, 0x00);
}
TEST(Protocol, RejectsWrongAddressChecksumAndLength) {
  std::vector<std::uint8_t> frame{0x3a, 1}; frame.push_back(checksum(4, frame));
  EXPECT_FALSE(mks_servo_can::parse_enable(5, frame).has_value());
  frame.back() ^= 1;
  EXPECT_FALSE(mks_servo_can::parse_enable(4, frame).has_value());
  EXPECT_FALSE(mks_servo_can::parse_enable(4, {0x3a, 1}).has_value());
}
TEST(Protocol, ParsesReadOnlyStatusValues) {
  std::vector<std::uint8_t> enable{0x3a, 1}; enable.push_back(checksum(1, enable));
  EXPECT_EQ(mks_servo_can::parse_enable(1, enable), true);
  std::vector<std::uint8_t> motor{0xf1, 4}; motor.push_back(checksum(1, motor));
  EXPECT_EQ(mks_servo_can::parse_motor_state(1, motor), 4);
  std::vector<std::uint8_t> stall{0x3e, 0}; stall.push_back(checksum(1, stall));
  EXPECT_EQ(mks_servo_can::parse_stall(1, stall), false);
  EXPECT_FALSE(mks_servo_can::supports_motor_state(2));
  EXPECT_TRUE(mks_servo_can::supports_motor_state(5));
}
TEST(Protocol, ManualJogAccelerationKeepsRampIndependentOfMicrostep) {
  using namespace mks_servo_can;
  EXPECT_EQ(manual_acceleration_code(16),1);EXPECT_EQ(manual_acceleration_code(32),1);EXPECT_EQ(manual_acceleration_code(128),224);
  EXPECT_EQ(manual_acceleration_code(0),1) << "nieznany mikrokrok: acc=1";
  // ~78 RPM/s niezależnie od mikrokroku (bez kompensacji 128 dawało ~9,8 RPM/s, dobieg JOG ~3 s z 30 RPM).
  EXPECT_NEAR(manual_ramp_rpm_s(16),78.43,.01);EXPECT_NEAR(manual_ramp_rpm_s(128),78.13,.01);
  EXPECT_EQ(speed_request(1,false,0,manual_acceleration_code(128))[3],224);
}
TEST(Protocol, ManualJogStopRampFollowsJointAccelerationLimit) {
  using namespace mks_servo_can;
  // 500 °/s² przegubu × przełożenie 10 = 833,3 RPM/s wału.
  const double limit = 500.0 * 10 / 6;
  EXPECT_EQ(manual_stop_acceleration_code(128,limit),253);
  EXPECT_NEAR(f6_ramp_rpm_s(128,253),833.3,.1);
  EXPECT_EQ(manual_stop_acceleration_code(16,limit),232);
  EXPECT_LE(f6_ramp_rpm_s(16,232),limit) << "zaokrąglenie nie może przekroczyć limitu";
  EXPECT_GT(f6_ramp_rpm_s(16,233),limit);
  // Nigdy wolniej niż start JOG; brak limitu lub mikrokroku → rampa startu.
  EXPECT_EQ(manual_stop_acceleration_code(128,10),224);
  EXPECT_EQ(manual_stop_acceleration_code(128,0),224);
  EXPECT_EQ(manual_stop_acceleration_code(0,limit),1);
  EXPECT_EQ(manual_stop_acceleration_code(128,1e9),255);
}
