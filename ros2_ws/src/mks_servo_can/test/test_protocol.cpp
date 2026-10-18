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
