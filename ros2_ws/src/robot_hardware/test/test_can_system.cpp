#include <gtest/gtest.h>
#include "pluginlib/class_loader.hpp"
#include "robot_hardware/can_system.hpp"

TEST(CanSystem, PluginLoadsAndRejectsMissingConfiguration) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader("hardware_interface", "hardware_interface::SystemInterface");
  const auto plugin = loader.createSharedInstance("robot_hardware/CanSystem");
  hardware_interface::HardwareInfo info;
  EXPECT_EQ(plugin->on_init(info), hardware_interface::CallbackReturn::ERROR);
}
hardware_interface::HardwareInfo example_info() {
  hardware_interface::HardwareInfo info;
  info.hardware_parameters = {{"can_interface", "test_only"}, {"motion_enabled", "false"},
    {"cycle_ms", "10"}, {"can_bitrate","1000000"}, {"feedback_timeout_ms", "60"}, {"command_timeout_ms", "60"}, {"response_timeout_ms", "20"}};
  for (unsigned i = 0; i < 6; ++i) {
    hardware_interface::ComponentInfo joint; joint.name = "joint_" + std::to_string(i + 1);
    hardware_interface::InterfaceInfo position; position.name = "position";
    joint.command_interfaces.push_back(position);joint.state_interfaces.push_back(position);
    hardware_interface::InterfaceInfo velocity;velocity.name="velocity";joint.command_interfaces.push_back(velocity);joint.state_interfaces.push_back(velocity);
    joint.parameters = {{"can_id", std::to_string(i + 1)}, {"gear_ratio", "1"}, {"direction", "1"}, {"zero_offset_rad", "0"},
      {"lower_rad", "-2"}, {"upper_rad", "2"}, {"velocity_rad_s", "1"}, {"acceleration_rad_s2", "100"},
      {"tracking_error_rad", "0.2"}, {"mks_acceleration_code", "2"}};
    info.joints.push_back(joint);
  }
  return info;
}
TEST(CanSystem, ExportsTwelveStateAndCommandInterfacesWithoutOpeningCan) {
  robot_hardware::CanSystem plugin;
  ASSERT_EQ(plugin.on_init(example_info()), hardware_interface::CallbackReturn::SUCCESS);
  const auto states = plugin.export_state_interfaces(); const auto commands = plugin.export_command_interfaces();
  ASSERT_EQ(states.size(), 12u); ASSERT_EQ(commands.size(), 12u);
  EXPECT_EQ(states[10].get_name(), "joint_6/position");
  EXPECT_EQ(commands[0].get_name(), "joint_1/position");
}
TEST(CanSystem, RejectsDuplicatesAndUnknownParametersInsteadOfGuessing) {
  robot_hardware::CanSystem plugin;
  auto info = example_info(); info.joints[5].parameters["can_id"] = "1";
  EXPECT_EQ(plugin.on_init(info), hardware_interface::CallbackReturn::ERROR);
  info = example_info(); info.joints[0].parameters["gear_ratio"] = "null";
  EXPECT_EQ(plugin.on_init(info), hardware_interface::CallbackReturn::ERROR);
}
TEST(CanSystem, PositionVelocityCommandsAreExportedWithoutCan) {
  auto info=example_info();
  robot_hardware::CanSystem plugin;ASSERT_EQ(plugin.on_init(info),hardware_interface::CallbackReturn::SUCCESS);
  const auto commands=plugin.export_command_interfaces();ASSERT_EQ(commands.size(),12u);
  EXPECT_EQ(commands[1].get_name(),"joint_1/velocity");EXPECT_EQ(commands[11].get_name(),"joint_6/velocity");
}
TEST(CanSystem, PositionCorrectionGainIsValidatedWithoutOpeningCan) {
  auto info=example_info();info.joints[0].parameters["position_kp"]="-1";
  robot_hardware::CanSystem rejected;EXPECT_EQ(rejected.on_init(info),hardware_interface::CallbackReturn::ERROR);
  info.joints[0].parameters["position_kp"]="2.5";
  robot_hardware::CanSystem accepted;EXPECT_EQ(accepted.on_init(info),hardware_interface::CallbackReturn::SUCCESS);
}

TEST(CanSystem, PositionAndVelocityStatesAndCommandsAreMandatory) {
  auto info=example_info();info.joints[0].state_interfaces.pop_back();robot_hardware::CanSystem plugin;
  EXPECT_EQ(plugin.on_init(info),hardware_interface::CallbackReturn::ERROR);
  info=example_info();info.joints[0].command_interfaces.pop_back();
  EXPECT_EQ(plugin.on_init(info),hardware_interface::CallbackReturn::ERROR);
}
TEST(CanSystem, JointLookaheadAndVelocityFeedForwardIncludeZeroVelocityHolding) {
  mks_servo_can::AxisConfig a{1,10,-1,.1,-2,2,1,100,.1,240};
  a.streaming.lookahead_time_sec=.02;a.streaming.velocity_ff_scale=1.5;a.streaming.min_holding_rpm=5;a.streaming.max_rpm=10;
  const auto moving=mks_servo_can::streaming_setpoint(a,.2,.1);
  EXPECT_NEAR(moving.lookahead_position,.202,1e-12);EXPECT_EQ(moving.rpm,10);EXPECT_LT(moving.ticks,0);
  const auto holding=mks_servo_can::streaming_setpoint(a,.2,0);
  EXPECT_DOUBLE_EQ(holding.lookahead_position,.2);EXPECT_EQ(holding.rpm,5);
}
