#include <gtest/gtest.h>
#include "robot_can_diagnostics/preview.hpp"
using namespace std::chrono_literals;
using namespace mks_servo_can;
TEST(Preview, OnlyFreshNewMeasurementsWithIndividualTimestamps) {
  ManualState state; std::array<Time, kAxes> last{}; const Time now{2s};
  for (unsigned i : {0, 1, 3}) {
    state.axes[i].available = true;
    state.axes[i].position = {i == 0 ? -450.0 : 720.0, now - std::chrono::milliseconds(i + 10), true};
  }
  const auto messages = robot_can_diagnostics::preview_states(state, now, rclcpp::Time(10'000'000'000LL), last);
  ASSERT_EQ(messages.size(), 3u);
  EXPECT_EQ(messages[0].name[0], "joint_1"); EXPECT_EQ(messages[2].name[0], "joint_4");
  EXPECT_NEAR(messages[0].position[0], -2.5 * std::acos(-1.0), 1e-12);
  EXPECT_NEAR(messages[2].position[0], 4 * std::acos(-1.0), 1e-12);
  EXPECT_EQ(rclcpp::Time(messages[0].header.stamp).nanoseconds(), 9'990'000'000LL);
  EXPECT_EQ(rclcpp::Time(messages[2].header.stamp).nanoseconds(), 9'987'000'000LL);
  EXPECT_TRUE(messages[0].velocity.empty()); EXPECT_TRUE(messages[0].effort.empty());
  EXPECT_TRUE(robot_can_diagnostics::preview_states(state, now, rclcpp::Time(10'000'000'000LL), last).empty());
  state.axes[0].position.stamp = now;
  EXPECT_EQ(robot_can_diagnostics::preview_states(state, now, rclcpp::Time(10'000'000'000LL), last).size(), 1u);
}
TEST(Preview, NeverInventMissingStaleOrFaultedPositions) {
  ManualState state; std::array<Time, kAxes> last{}; const Time now{2s};
  EXPECT_TRUE(robot_can_diagnostics::preview_states(state, now, rclcpp::Time(10'000'000'000LL), last).empty());
  state.axes[0].available = true; state.axes[0].position = {30, now - 250ms, true};
  EXPECT_TRUE(robot_can_diagnostics::preview_states(state, now, rclcpp::Time(10'000'000'000LL), last).empty());
  state.axes[0].position.stamp = now + 1ms;
  EXPECT_TRUE(robot_can_diagnostics::preview_states(state, now, rclcpp::Time(10'000'000'000LL), last).empty());
  state.axes[0].position.stamp = now; state.fault = true;
  EXPECT_TRUE(robot_can_diagnostics::preview_states(state, now, rclcpp::Time(10'000'000'000LL), last).empty());
}
TEST(Preview, ConfiguredJointUsesConvertedPositionAndUncalibratedJointIsOmitted) {
  ManualState state;std::array<Time,kAxes> last{};const Time now{2s};
  auto &axis=state.axes[0];axis.available=true;axis.configured=true;axis.position={720,now,true};
  EXPECT_TRUE(robot_can_diagnostics::preview_states(state,now,rclcpp::Time(10'000'000'000LL),last).empty());
  axis.joint_position={-.2,now,true};
  const auto messages=robot_can_diagnostics::preview_states(state,now,rclcpp::Time(10'000'000'000LL),last);
  ASSERT_EQ(messages.size(),1u);EXPECT_DOUBLE_EQ(messages[0].position[0],-.2);
}
