#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include <cmath>
#include <algorithm>

namespace mks_servo_can {
TcpStreamingSnapshot CanBusManager::tcp_streaming_snapshot() const {
  TcpStreamingSnapshot out;out.reference=committed_positions_;out.reference_created=last_command_;out.generation=control_generation_;
  out.command_available=source_==Source::tcp_jog && last_command_>tcp_started_;
  for(unsigned j=0;j<6;++j) {
    const auto &data=stream_payloads_[j];const auto raw=(std::uint32_t(data[4])<<16)|(std::uint32_t(data[5])<<8)|data[6];
    const auto ticks=raw&0x800000 ? static_cast<std::int32_t>(raw)-0x1000000 : static_cast<std::int32_t>(raw);
    out.target[j]=position_to_joint(config_.axes[j],ticks);
    out.shaft_rpm[j]=(((data[1]&0x0f)<<8)|data[2])*speed_scale(devices_[config_.axes[j].address].microstep.value);
  }
  return out;
}
bool CanBusManager::tcp_jog_available(Time now) const {
  if(!config_.tcp_jog_enabled || !configuration_ready_ || configuration_locked_ || scanning_ || active_ || fault_ ||
      manual_->busy() || executor_.status.busy || executor_.status.state=="ready" || !transport_->is_open() || !feedback_good(now) ||
      config_.bitrate!=1000000 || streaming_bus_utilization(6,config_.bitrate)>std::min(kPlannedUtilization,config_.utilization_limit)+1e-12) return false;
  for(unsigned j=0;j<6;++j) {
    const auto &device=devices_[config_.axes[j].address];
    if(!profile_.axes[j].configured || !manual_->calibration_valid(j+1) || !device.mode.valid || device.mode.value!=5 ||
        !device.microstep.valid || !device.responses_disabled || !device.synchronization_disabled) return false;
  }
  return true;
}
std::uint64_t CanBusManager::begin_tcp_jog(const std::string &session,const std::string &revision,
    const std::string &model,std::uint64_t sequence,Time issued,Time now) {
  if(!tcp_jog_available(now) || session.empty() || session.size()>64 || !sequence || issued>now ||
      now-issued>=std::chrono::milliseconds(100) || issued<=tcp_last_stop_ || revision!=profile_.revision ||
      model.empty() || model!=motion_model_ || config_.bitrate!=1000000 ||
      streaming_bus_utilization(6,config_.bitrate)>std::min(kPlannedUtilization,config_.utilization_limit)+1e-12 || !tcp_owner_.empty()) return 0;
  for(unsigned j=0;j<6;++j) if(!profile_.axes[j].configured || !manual_->calibration_valid(j+1) ||
      !devices_[config_.axes[j].address].mode.valid || devices_[config_.axes[j].address].mode.value!=5) return 0;
  if(program_owned_ && !active_ && !executor_.status.busy && executor_.status.state!="ready") program_owned_=false;
  if(!activate(Source::tcp_jog,now)) return 0;
  for(unsigned j=0;j<6;++j) config_.axes[j].streaming=config_.program_streaming[j];
  tcp_owner_=session;tcp_sequence_=sequence;tcp_lease_=issued;tcp_started_=now;tcp_braking_=false;
  ++control_generation_;return control_generation_;
}
bool CanBusManager::tcp_jog_control(const std::string &session,std::uint64_t generation,std::uint64_t sequence,
    Time issued,Time now,bool brake) {
  if(!active_ || source_!=Source::tcp_jog || session!=tcp_owner_ || generation!=control_generation_ ||
      sequence<=tcp_sequence_ || issued>now || now-issued>=std::chrono::milliseconds(100)) return false;
  tcp_sequence_=sequence;tcp_lease_=issued;tcp_braking_=tcp_braking_ || brake;return true;
}
bool CanBusManager::brake_tcp_jog(const std::string &session,std::uint64_t generation) {
  if(!active_ || source_!=Source::tcp_jog || session!=tcp_owner_ || generation!=control_generation_) return false;
  tcp_braking_=true;return true;
}
bool CanBusManager::finish_tcp_jog(const std::string &session,std::uint64_t generation) {
  if(!active_ || source_!=Source::tcp_jog || session!=tcp_owner_ || generation!=control_generation_) return false;
  // Wyłącznie po wysłaniu referencji v=0. Potwierdzenie pomiarowe zapewnia adapter.
  if(!tcp_braking_ || latest_) return false;
  for(double v:previous_velocity_) if(std::abs(v)>1e-9) return false;
  active_=false;source_=Source::none;tcp_owner_.clear();tcp_last_stop_=last_tick_;
  message_="JOG TCP: zatrzymanie potwierdzone";return true;
}
}
