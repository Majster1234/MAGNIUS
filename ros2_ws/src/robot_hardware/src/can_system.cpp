#include "robot_hardware/can_system.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
#include "pluginlib/class_list_macros.hpp"

namespace robot_hardware {
namespace {
double required(const std::unordered_map<std::string, std::string> &parameters, const std::string &key) {
  const auto &text = parameters.at(key); std::size_t end{};
  const auto value = std::stod(text, &end);
  if (end != text.size() || !std::isfinite(value)) throw std::invalid_argument("Nieprawidłowy parametr: " + key);
  return value;
}
unsigned integer(const std::unordered_map<std::string, std::string> &p, const std::string &key, unsigned upper) {
  const auto number = required(p, key);
  if (number < 1 || number > upper || std::floor(number) != number) throw std::invalid_argument("Nieprawidłowy parametr całkowity: " + key);
  return static_cast<unsigned>(number);
}
bool verified(const std::unordered_map<std::string, std::string> &p, const std::string &key) {
  const auto it = p.find(key); return it != p.end() && it->second == "true";
}
}
CanSystem::~CanSystem() { shutdown(); }
hardware_interface::CallbackReturn CanSystem::on_init(const hardware_interface::HardwareInfo &info) {
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;
  try {
    if (info_.joints.size() != mks_servo_can::kAxes) throw std::invalid_argument("Wymagane sześć osi joint_1–joint_6");
    const auto &p = info_.hardware_parameters;
    config_.interface = p.at("can_interface");
    config_.motion_enabled = verified(p, "motion_enabled");
    config_.commissioning_verified = verified(p, "commissioning_verified");
    config_.synchronization_verified = verified(p, "synchronization_verified");
    config_.stop_verified = verified(p, "stop_verified");
    config_.source_lease_required = true;
    config_.cycle = std::chrono::milliseconds(integer(p, "cycle_ms", 1000));
    config_.feedback_timeout = std::chrono::milliseconds(integer(p, "feedback_timeout_ms", 1000));
    config_.command_timeout = std::chrono::milliseconds(integer(p, "command_timeout_ms", 1000));
    config_.response_timeout = std::chrono::milliseconds(integer(p, "response_timeout_ms", 1000));
    for (std::size_t i = 0; i < mks_servo_can::kAxes; ++i) {
      const auto &joint = info_.joints[i]; const auto &a = joint.parameters;
      if (joint.name != "joint_" + std::to_string(i + 1) || joint.command_interfaces.size() != 1 ||
          joint.command_interfaces[0].name != "position" || joint.state_interfaces.size() != 1 ||
          joint.state_interfaces[0].name != "position") throw std::invalid_argument("Wymagane interfejsy position i nazwy joint_1–joint_6");
      config_.axes[i] = {static_cast<std::uint16_t>(integer(a, "can_id", 2047)), required(a, "gear_ratio"), required(a, "direction"),
          required(a, "zero_offset_rad"), required(a, "lower_rad"), required(a, "upper_rad"), required(a, "velocity_rad_s"),
          required(a, "acceleration_rad_s2"), required(a, "tracking_error_rad"), static_cast<std::uint8_t>(integer(a, "mks_acceleration_code", 255))};
    }
    runtime_ = std::make_shared<mks_servo_can::BusRuntime>(config_);
    positions_.fill(std::numeric_limits<double>::quiet_NaN()); commands_ = positions_;
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_hardware"), "Niepełna konfiguracja sprzętu: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}
hardware_interface::CallbackReturn CanSystem::on_configure(const rclcpp_lifecycle::State &) {
  try {
    runtime_->reset_session();
    runtime_->start(false);
    if (!runtime_->snapshot().open) { runtime_->shutdown(); return hardware_interface::CallbackReturn::ERROR; }
    diagnostics_ = std::make_shared<robot_can_diagnostics::DiagnosticsNode>(runtime_, config_);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(diagnostics_);
    ros_thread_ = std::thread([this] { executor_->spin(); });
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_hardware"), "%s", e.what()); shutdown();
    return hardware_interface::CallbackReturn::ERROR;
  }
}
hardware_interface::CallbackReturn CanSystem::on_activate(const rclcpp_lifecycle::State &) {
  if (!runtime_->activate(mks_servo_can::Source::trajectory)) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_hardware"), "Aktywacja odrzucona: commissioning, synchronizacja, stop, tryb CAN, ENABLE lub świeżość pozycji");
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Callback cyklu życia nie jest częścią read/write. Pobieramy kompletny stan.
  for (unsigned attempt = 0; attempt < 20; ++attempt) {
    if (runtime_->try_read(state_)) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!state_.active) { runtime_->stop("Nie udało się pobrać stanu aktywacji"); return hardware_interface::CallbackReturn::ERROR; }
  for (std::size_t i = 0; i < mks_servo_can::kAxes; ++i) positions_[i] = state_.positions[i].value;
  commands_ = positions_; active_ = true; sequence_ = 0; return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn CanSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  active_ = false; runtime_->stop("Dezaktywacja kontrolera — wymagany reset przed wznowieniem");
  return hardware_interface::CallbackReturn::SUCCESS;
}
void CanSystem::shutdown() {
  active_ = false;
  if (executor_) executor_->cancel();
  if (ros_thread_.joinable()) ros_thread_.join();
  diagnostics_.reset(); executor_.reset();
  if (runtime_) runtime_->shutdown();
}
hardware_interface::CallbackReturn CanSystem::on_cleanup(const rclcpp_lifecycle::State &) { shutdown(); return hardware_interface::CallbackReturn::SUCCESS; }
hardware_interface::CallbackReturn CanSystem::on_shutdown(const rclcpp_lifecycle::State &) { shutdown(); return hardware_interface::CallbackReturn::SUCCESS; }
hardware_interface::CallbackReturn CanSystem::on_error(const rclcpp_lifecycle::State &) { shutdown(); return hardware_interface::CallbackReturn::SUCCESS; }
std::vector<hardware_interface::StateInterface> CanSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> result;
  for (std::size_t i = 0; i < positions_.size(); ++i) result.emplace_back(info_.joints[i].name, "position", &positions_[i]);
  return result;
}
std::vector<hardware_interface::CommandInterface> CanSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> result;
  for (std::size_t i = 0; i < commands_.size(); ++i) result.emplace_back(info_.joints[i].name, "position", &commands_[i]);
  return result;
}
hardware_interface::return_type CanSystem::read(const rclcpp::Time &, const rclcpp::Duration &) {
  runtime_->try_read(state_);
  const auto now = mks_servo_can::Clock::now();
  if (active_ && (state_.fault || !state_.active)) return hardware_interface::return_type::ERROR;
  for (std::size_t i = 0; i < positions_.size(); ++i) {
    if (!state_.positions[i].fresh(now, config_.feedback_timeout)) {
      if (active_) return hardware_interface::return_type::ERROR;
    } else positions_[i] = state_.positions[i].value;
  }
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type CanSystem::write(const rclcpp::Time &, const rclcpp::Duration &) {
  if (!active_) return hardware_interface::return_type::OK;
  if (state_.fault) return hardware_interface::return_type::ERROR;
  mks_servo_can::CommandBatch batch; batch.positions = commands_; batch.sequence = ++sequence_;
  batch.created = mks_servo_can::Clock::now(); batch.expires = batch.created + config_.cycle;
  batch.source = state_.source;
  // Przy zajętym mutexie pomijamy próbkę; watchdog działa w niezależnym wątku IO.
  runtime_->try_submit(batch);
  return hardware_interface::return_type::OK;
}
}  // namespace robot_hardware
PLUGINLIB_EXPORT_CLASS(robot_hardware::CanSystem, hardware_interface::SystemInterface)
