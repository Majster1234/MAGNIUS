#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <future>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

#include <linux/can/error.h>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "mks_servo_can/protocol.hpp"
#include "mks_servo_can/socketcan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

namespace {
std::string quote(const std::string &value) {
  std::string result{"\""};
  for (const auto character : value) {
    if (character == '\\' || character == '\"') result += '\\';
    result += character;
  }
  return result + "\"";
}
std::string json_bool(bool value) { return value ? "true" : "false"; }
std::string json_number(double value) {
  std::ostringstream result; result << std::fixed << std::setprecision(3) << value; return result.str();
}
int restart_can_service() {
  const auto pid = ::fork();
  if (pid < 0) return -1;
  if (pid == 0) {
    ::execl("/usr/bin/sudo", "sudo", "-n", "/usr/bin/systemctl", "restart",
            "robot-can@can0.service", static_cast<char *>(nullptr));
    _exit(127);
  }
  int status{};
  return ::waitpid(pid, &status, 0) == pid && WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}
}  // namespace

class CanDiagnostics final : public rclcpp::Node {
 public:
  CanDiagnostics() : Node("robot_can_diagnostics") {
    interface_ = declare_parameter<std::string>("can_interface", "can0");
    bitrate_ = declare_parameter<int>("bitrate", 500000);
    timeout_ = std::chrono::milliseconds(declare_parameter<int>("response_timeout_ms", 20));
    publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/robot/can/diagnostics", rclcpp::QoS(1));
    scan_service_ = create_service<std_srvs::srv::Trigger>(
        "/robot/can/scan", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          start_scan_response(*response);
        });
    reset_service_ = create_service<std_srvs::srv::Trigger>(
        "/robot/can/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          reset_response(*response);
        });
    tick_timer_ = create_wall_timer(5ms, [this] { tick(); });
    publish_timer_ = create_wall_timer(200ms, [this] { publish(); });
    open_socket();
    if (socket_.is_open()) start_scan();
  }

 private:
  enum class RequestKind { identity, mode, enable, motor_state, stall };
  struct Pending { std::uint16_t address; RequestKind kind; Clock::time_point sent; };
  struct Device {
    mks_servo_can::Identity identity;
    bool online{true};
    std::optional<bool> enabled;
    std::optional<bool> stall;
    std::optional<std::uint8_t> mode;
    std::optional<std::uint8_t> motor_state;
    std::uint8_t next_read{};
    unsigned failed_reads{};
    unsigned timeouts{};
    unsigned invalid_responses{};
    unsigned received{};
    double rtt_sum_ms{};
    double rtt_min_ms{};
    double rtt_max_ms{};
    double last_rtt_ms{};
    double round_start_ms{};
    double full_round_ms{};
    Clock::time_point next_due{Clock::now()};
    Clock::time_point last_ok{Clock::now()};
  };

  void open_socket() {
    try {
      socket_.open(interface_);
      interface_error_.clear(); bus_off_ = false;
    } catch (const std::exception &error) {
      interface_error_ = error.what();
    }
  }
  void start_scan() {
    scanning_ = true; scan_address_ = 1; scan_checked_ = 0; operation_ = "Skanowanie";
    operation_message_ = "Skanowanie adresów 1–2047";
  }
  void start_scan_response(std_srvs::srv::Trigger::Response &response) {
    if (resetting_ || scanning_) {
      response.success = false; response.message = resetting_ ? "Reset CAN jest w toku" : "Skanowanie już trwa"; return;
    }
    if (!socket_.is_open() || bus_off_) {
      response.success = false; response.message = "Interfejs CAN nie jest gotowy; użyj Reset CAN"; return;
    }
    start_scan(); response.success = true; response.message = "Skanowanie przyjęte";
  }
  void reset_response(std_srvs::srv::Trigger::Response &response) {
    if (resetting_) { response.success = false; response.message = "Reset CAN już trwa"; return; }
    resetting_ = true; scanning_ = false; pending_.clear(); socket_.close();
    operation_ = "Resetowanie"; operation_message_ = "Restart robot-can@can0.service";
    reset_future_ = std::async(std::launch::async, restart_can_service);
    response.success = true; response.message = "Reset CAN przyjęty";
  }
  bool rate_available(Clock::time_point now) {
    while (!sent_times_.empty() && now - sent_times_.front() >= 1s) sent_times_.erase(sent_times_.begin());
    return sent_times_.size() < 200;
  }
  bool has_pending(std::uint16_t address) const {
    for (const auto &request : pending_) if (request.address == address) return true;
    return false;
  }
  void send(std::uint16_t address, RequestKind kind, Clock::time_point now) {
    if (pending_.size() >= 4 || !rate_available(now)) return;
    try {
      std::vector<std::uint8_t> frame;
      switch (kind) {
        case RequestKind::identity: frame = mks_servo_can::request(address, 0x40); break;
        case RequestKind::mode: frame = mks_servo_can::mode_request(address); break;
        case RequestKind::enable: frame = mks_servo_can::request(address, 0x3a); break;
        case RequestKind::motor_state: frame = mks_servo_can::request(address, 0xf1); break;
        case RequestKind::stall: frame = mks_servo_can::request(address, 0x3e); break;
      }
      socket_.send(address, frame); pending_.push_back({address, kind, now}); sent_times_.push_back(now); ++tx_frames_;
    } catch (const std::exception &error) { interface_error_ = error.what(); ++errors_; }
  }
  void tick() {
    const auto now = Clock::now();
    finish_reset();
    if (resetting_) return;
    receive_all(now); expire_requests(now);
    if (!socket_.is_open() || bus_off_) return;
    // Known devices are always selected before the next unexplored address.
    for (std::uint16_t address = 1; address <= mks_servo_can::kMaximumAddress; ++address) {
      auto &record = devices_[address];
      if (!record || record->identity.model != mks_servo_can::Model::kServo42D ||
          has_pending(address) || now < record->next_due) continue;
      const auto period = record->online ? 66ms : 1s;
      RequestKind kind = RequestKind::enable;
      if (record->next_read == 1 && record->mode && mks_servo_can::supports_motor_state(*record->mode)) kind = RequestKind::motor_state;
      if (record->next_read == 2 || (record->next_read == 1 && (!record->mode || !mks_servo_can::supports_motor_state(*record->mode)))) kind = RequestKind::stall;
      send(address, kind, now); record->next_due = now + period; return;
    }
    if (scanning_ && scan_address_ <= mks_servo_can::kMaximumAddress && pending_.empty()) {
      send(scan_address_, RequestKind::identity, now);
    }
  }
  void receive_all(Clock::time_point now) {
    try {
      while (true) {
        auto frame = socket_.receive(); if (!frame) break;
        if (frame->error) {
          ++errors_;
          if (frame->error_flags & CAN_ERR_BUSOFF) { bus_off_ = true; operation_ = "Wstrzymano"; operation_message_ = "CAN BUS-OFF — wymagany Reset CAN"; }
          continue;
        }
        ++rx_frames_;
        auto match = pending_.end();
        for (auto it = pending_.begin(); it != pending_.end(); ++it) if (it->address == frame->address) { match = it; break; }
        if (match == pending_.end()) { ++invalid_responses_; continue; }
        const auto elapsed = std::chrono::duration<double, std::milli>(now - match->sent).count();
        const auto kind = match->kind; pending_.erase(match);
        accept_response(frame->address, kind, frame->data, elapsed, now);
      }
    } catch (const std::exception &error) { interface_error_ = error.what(); ++errors_; }
  }
  void expire_requests(Clock::time_point now) {
    for (auto it = pending_.begin(); it != pending_.end();) {
      if (now - it->sent < timeout_) { ++it; continue; }
      const auto request = *it; it = pending_.erase(it); ++timeouts_;
      if (request.kind == RequestKind::identity) { advance_scan(request.address); continue; }
      auto &record = devices_[request.address];
      if (record) {
        ++record->timeouts;
        if (++record->failed_reads >= 3) { record->online = false; record->enabled.reset(); record->motor_state.reset(); }
      }
    }
  }
  void accept_response(std::uint16_t address, RequestKind kind, const std::vector<std::uint8_t> &data,
                       double rtt, Clock::time_point now) {
    if (kind == RequestKind::identity) {
      const auto identity = mks_servo_can::parse_identity(address, data);
      advance_scan(address);
      if (!identity) { ++invalid_responses_; return; }
      auto &record = devices_[address];
      if (!record) record.emplace();
      record->identity = *identity; record->online = true; record->next_due = now;
      record->last_ok = now; update_rtt(*record, rtt);
      if (identity->model == mks_servo_can::Model::kServo42D) send(address, RequestKind::mode, now);
      return;
    }
    auto &record = devices_[address];
    if (!record) { ++invalid_responses_; return; }
    bool valid = false;
    if (kind == RequestKind::mode) { const auto value = mks_servo_can::parse_work_mode(address, data); if (value) { record->mode = *value; valid = true; } }
    if (kind == RequestKind::enable) { const auto value = mks_servo_can::parse_enable(address, data); if (value) { record->enabled = *value; valid = true; } }
    if (kind == RequestKind::motor_state) { const auto value = mks_servo_can::parse_motor_state(address, data); if (value) { record->motor_state = *value; valid = true; } }
    if (kind == RequestKind::stall) { const auto value = mks_servo_can::parse_stall(address, data); if (value) { record->stall = *value; valid = true; } }
    if (!valid) { ++record->invalid_responses; ++invalid_responses_; return; }
    record->online = true; record->failed_reads = 0; record->last_ok = now; update_rtt(*record, rtt);
    if (kind == RequestKind::enable) record->next_read = 1;
    else if (kind == RequestKind::motor_state) record->next_read = 2;
    else if (kind == RequestKind::stall) { record->next_read = 0; const auto start = record->round_start_ms; const auto current = std::chrono::duration<double, std::milli>(now.time_since_epoch()).count(); record->full_round_ms = start > 0 ? current - start : 0; record->round_start_ms = current; }
  }
  void update_rtt(Device &record, double rtt) {
    record.last_rtt_ms = rtt; record.rtt_sum_ms += rtt; ++record.received;
    record.rtt_min_ms = record.received == 1 ? rtt : std::min(record.rtt_min_ms, rtt);
    record.rtt_max_ms = std::max(record.rtt_max_ms, rtt);
  }
  void advance_scan(std::uint16_t address) {
    if (!scanning_ || address != scan_address_) return;
    ++scan_checked_; ++scan_address_;
    if (scan_address_ > mks_servo_can::kMaximumAddress) { scanning_ = false; operation_ = "Gotowe"; operation_message_ = "Skanowanie zakończone"; }
  }
  void finish_reset() {
    if (!resetting_ || reset_future_.wait_for(0ms) != std::future_status::ready) return;
    const auto result = reset_future_.get(); resetting_ = false;
    if (result != 0) { interface_error_ = "Brak uprawnienia lub błąd restartu robot-can@can0.service"; operation_ = "Błąd resetu"; operation_message_ = interface_error_; ++errors_; return; }
    clear_session(); open_socket();
    if (socket_.is_open()) start_scan(); else { operation_ = "Błąd resetu"; operation_message_ = interface_error_; }
  }
  void clear_session() { devices_.fill(std::nullopt); pending_.clear(); sent_times_.clear(); tx_frames_ = rx_frames_ = errors_ = timeouts_ = invalid_responses_ = 0; scan_checked_ = 0; }
  diagnostic_msgs::msg::DiagnosticStatus status(const std::string &name, std::uint8_t level,
                                                 const std::string &message,
                                                 std::vector<diagnostic_msgs::msg::KeyValue> values) {
    diagnostic_msgs::msg::DiagnosticStatus result; result.name = name; result.level = level; result.message = message; result.hardware_id = interface_; result.values = std::move(values); return result;
  }
  static diagnostic_msgs::msg::KeyValue value(const std::string &key, const std::string &json) {
    diagnostic_msgs::msg::KeyValue result; result.key = key; result.value = json; return result;
  }
  void publish() {
    diagnostic_msgs::msg::DiagnosticArray message; message.header.stamp = get_clock()->now();
    std::size_t count{}, online{}; double max_round{};
    for (const auto &record : devices_) if (record) { ++count; if (record->online) ++online; max_round = std::max(max_round, record->full_round_ms); }
    const auto level = bus_off_ || !interface_error_.empty() ? 2 : 0;
    const auto bus_state = bus_off_ ? "BUS-OFF" : socket_.is_open() ? "AKTYWNA" : "NIEDOSTĘPNA";
    message.status.push_back(status("can", level, bus_state, {
        value("interface", quote(interface_)), value("bitrate", std::to_string(bitrate_)), value("bus_state", quote(bus_state)),
        value("socket_open", json_bool(socket_.is_open())), value("device_count", std::to_string(count)), value("online_count", std::to_string(online)),
        value("full_round_ms", json_number(max_round)), value("errors", std::to_string(errors_)), value("timeouts", std::to_string(timeouts_)),
        value("invalid_responses", std::to_string(invalid_responses_)), value("rx_frames", std::to_string(rx_frames_)), value("tx_frames", std::to_string(tx_frames_)),
        value("sampled_at", quote(""))}));
    message.status.push_back(status("scan", resetting_ ? 1 : 0, operation_message_, {
        value("state", quote(operation_)), value("scanning", json_bool(scanning_)), value("resetting", json_bool(resetting_)),
        value("checked", std::to_string(scan_checked_)), value("total", "2047"), value("error", quote(interface_error_)), value("sampled_at", quote(""))}));
    for (std::uint16_t address = 1; address <= mks_servo_can::kMaximumAddress; ++address) {
      const auto &record = devices_[address]; if (!record) continue;
      const auto state_text = record->mode && mks_servo_can::supports_motor_state(*record->mode) && record->motor_state ? mks_servo_can::motor_state_name(*record->motor_state) : "Niedostępny";
      const auto enable = record->enabled ? json_bool(*record->enabled) : "null";
      const auto stall = record->stall ? json_bool(*record->stall) : "null";
      const auto average = record->received ? record->rtt_sum_ms / record->received : 0;
      const auto last = std::chrono::duration<double>(Clock::now() - record->last_ok).count();
      message.status.push_back(status("device/" + std::to_string(address), record->online ? 0 : 1,
          record->online ? "Odpowiada" : "Brak odpowiedzi", {
          value("address", std::to_string(address)), value("model", quote(mks_servo_can::model_name(record->identity.model))), value("online", json_bool(record->online)),
          value("enable", enable), value("motor_state", quote(state_text)), value("mode", quote(record->mode ? mks_servo_can::work_mode_name(*record->mode) : "Niedostępny")), value("stall", stall),
          value("rtt_ms", json_number(record->last_rtt_ms)), value("rtt_min_ms", json_number(record->rtt_min_ms)), value("rtt_max_ms", json_number(record->rtt_max_ms)), value("rtt_mean_ms", json_number(average)),
          value("full_round_ms", json_number(record->full_round_ms)), value("timeouts", std::to_string(record->timeouts)), value("invalid_responses", std::to_string(record->invalid_responses)), value("last_ok_age_seconds", json_number(last)), value("sampled_at", quote(""))}));
    }
    publisher_->publish(message);
  }
  std::string interface_; int bitrate_{}; std::chrono::milliseconds timeout_{20};
  mks_servo_can::SocketCan socket_; std::array<std::optional<Device>, 2048> devices_{}; std::vector<Pending> pending_; std::vector<Clock::time_point> sent_times_;
  bool scanning_{false}; bool resetting_{false}; bool bus_off_{false}; std::uint16_t scan_address_{1}; unsigned scan_checked_{};
  std::string operation_{"Oczekiwanie"}; std::string operation_message_{"Oczekiwanie na magistralę CAN"}; std::string interface_error_;
  std::future<int> reset_future_; std::uint64_t rx_frames_{}, tx_frames_{}, errors_{}, timeouts_{}, invalid_responses_{};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_; rclcpp::TimerBase::SharedPtr tick_timer_, publish_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr scan_service_, reset_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<CanDiagnostics>()); rclcpp::shutdown(); return 0;
}
