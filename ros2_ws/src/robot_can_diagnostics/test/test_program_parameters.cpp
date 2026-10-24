#include <gtest/gtest.h>
#include "robot_can_diagnostics/program_api.hpp"

namespace {
// Test parametryzacji API ROS nigdy nie otwiera ani nie uruchamia transportu.
class NoIo final : public mks_servo_can::Transport {
 public:
  void open(const std::string &) override { ADD_FAILURE()<<"Nie wolno otwierać CAN w teście parametrów"; }
  void close() override {}
  bool is_open() const override { return false; }
  void send(std::uint16_t,const std::vector<std::uint8_t> &) override { ADD_FAILURE()<<"Nie wolno nadawać"; }
  std::optional<mks_servo_can::Frame> receive() override { return std::nullopt; }
};
class ProgramParameters : public ::testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0,nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
  std::shared_ptr<mks_servo_can::BusRuntime> runtime() {
    return std::make_shared<mks_servo_can::BusRuntime>(mks_servo_can::BusConfig{},std::make_unique<NoIo>());
  }
};
}
TEST_F(ProgramParameters, DefaultPolicyConstructsWithoutAcceptanceFlagsAndWithoutIo) {
  auto node=std::make_shared<rclcpp::Node>("ptp_parameters_test");auto bus=runtime();
  EXPECT_NO_THROW(robot_can_diagnostics::ProgramApi api(node.get(),bus));
  EXPECT_EQ(node->get_parameter("execution_ack_timeout_ms").as_int(),10);
  EXPECT_EQ(node->get_parameter("execution_position_period_ms").as_int(),10);
  const auto tolerance=node->get_parameter("execution_position_tolerance_rad").as_double_array();
  ASSERT_EQ(tolerance.size(),6u);EXPECT_DOUBLE_EQ(tolerance[0],mks_servo_can::kProgramPositionTolerance);
}
TEST_F(ProgramParameters, LegacyAckDeadlineDoesNotCreateMotionAckGate) {
  auto options=rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("execution_ack_timeout_ms",20)});
  auto node=std::make_shared<rclcpp::Node>("ptp_bad_deadline",options);auto bus=runtime();
  EXPECT_NO_THROW(robot_can_diagnostics::ProgramApi api(node.get(),bus));
}
TEST_F(ProgramParameters, ToleranceMustContainSixPositiveFiniteValues) {
  for(const auto &values:{std::vector<double>{.1},std::vector<double>{.1,.1,.1,.1,.1,0}}) {
    auto options=rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("execution_position_tolerance_rad",values)});
    auto node=std::make_shared<rclcpp::Node>("ptp_bad_tolerance",options);auto bus=runtime();
    EXPECT_THROW(robot_can_diagnostics::ProgramApi api(node.get(),bus),std::invalid_argument);
  }
}

TEST_F(ProgramParameters, ExplicitMotionAckIsRejectedForStreaming) {
  auto options=rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("execution_use_ack",true)});
  auto node=std::make_shared<rclcpp::Node>("ptp_no_ack_test",options);auto bus=runtime();
  EXPECT_THROW(robot_can_diagnostics::ProgramApi api(node.get(),bus),std::invalid_argument);
}
