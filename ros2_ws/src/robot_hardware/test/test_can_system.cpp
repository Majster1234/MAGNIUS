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
    {"cycle_ms", "20"}, {"feedback_timeout_ms", "60"}, {"command_timeout_ms", "60"}, {"response_timeout_ms", "20"}};
  for (unsigned i = 0; i < 6; ++i) {
    hardware_interface::ComponentInfo joint; joint.name = "joint_" + std::to_string(i + 1);
    hardware_interface::InterfaceInfo position; position.name = "position";
    joint.command_interfaces.push_back(position); joint.state_interfaces.push_back(position);
    joint.parameters = {{"can_id", std::to_string(i + 1)}, {"gear_ratio", "1"}, {"direction", "1"}, {"zero_offset_rad", "0"},
      {"lower_rad", "-2"}, {"upper_rad", "2"}, {"velocity_rad_s", "1"}, {"acceleration_rad_s2", "100"},
      {"tracking_error_rad", "0.2"}, {"mks_acceleration_code", "2"}};
    info.joints.push_back(joint);
  }
  return info;
}
TEST(CanSystem, ExportsExactlySixPositionInterfacesWithoutOpeningCan) {
  robot_hardware::CanSystem plugin;
  ASSERT_EQ(plugin.on_init(example_info()), hardware_interface::CallbackReturn::SUCCESS);
  const auto states = plugin.export_state_interfaces(); const auto commands = plugin.export_command_interfaces();
  ASSERT_EQ(states.size(), 6u); ASSERT_EQ(commands.size(), 6u);
  EXPECT_EQ(states[5].get_name(), "joint_6/position");
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
  for(auto &joint:info.joints) { hardware_interface::InterfaceInfo velocity;velocity.name="velocity";joint.command_interfaces.push_back(velocity); }
  robot_hardware::CanSystem plugin;ASSERT_EQ(plugin.on_init(info),hardware_interface::CallbackReturn::SUCCESS);
  const auto commands=plugin.export_command_interfaces();ASSERT_EQ(commands.size(),12u);
  EXPECT_EQ(commands[1].get_name(),"joint_1/velocity");EXPECT_EQ(commands[11].get_name(),"joint_6/velocity");
}
