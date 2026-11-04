#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include "mks_servo_can/diagnostic_message.hpp"
#include "mks_servo_can/trace.hpp"
#include <cmath>
#include <algorithm>

namespace mks_servo_can {
using namespace std::chrono_literals;
namespace {
constexpr double kPi=3.14159265358979323846;
// Oś czasu rusza dopiero z zapasem kolejki; wcześniej napędy trzymają pozycję startową.
constexpr double kTcpStartBuffer=.05;
// Generator nie może zobowiązać ruchu dalej niż na tyle do przodu.
constexpr double kTcpMaxHorizon=1.;
double seconds(Clock::duration d) {return std::chrono::duration<double>(d).count();}
}
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
  // Te same parametry F5 co PTP/LIN (lookahead, FF, Kp, dithering, acc).
  for(unsigned j=0;j<6;++j) config_.axes[j].streaming=config_.program_streaming[j];
  tcp_owner_=session;tcp_sequence_=sequence;tcp_lease_=issued;tcp_started_=now;tcp_slot_=0;
  tcp_points_.clear();tcp_origin_={};tcp_end_=-1;tcp_closed_=tcp_done_=false;tcp_reason_.clear();tcp_final_started_={};
  ++control_generation_;return control_generation_;
}
Time CanBusManager::tcp_slot_end(Time now) const {
  return tcp_started_+((now-tcp_started_)/kStreamingCycle+1)*kStreamingCycle;
}
bool CanBusManager::tcp_jog_control(const std::string &session,std::uint64_t generation,std::uint64_t sequence,
    Time issued,Time now) {
  if(!active_ || source_!=Source::tcp_jog || session!=tcp_owner_ || generation!=control_generation_ ||
      sequence<=tcp_sequence_ || issued>now || now-issued>=std::chrono::milliseconds(100)) return false;
  tcp_sequence_=sequence;tcp_lease_=issued;return true;
}
bool CanBusManager::append_tcp_points(const std::string &session,std::uint64_t generation,const std::vector<ProgramPoint> &points,bool closed,Time now) {
  if(!active_ || fault_ || source_!=Source::tcp_jog || session!=tcp_owner_ || generation!=control_generation_ || tcp_end_>=0 || tcp_closed_) return false;
  double previous=tcp_points_.empty() ? -1 : tcp_points_.back().seconds;
  for(const auto &p:points) {
    if(!std::isfinite(p.seconds) || (previous<0 ? p.seconds!=0 : (p.seconds<=previous+1e-6 || p.seconds>previous+.05))) return false;
    for(unsigned j=0;j<kAxes;++j) {
      const auto &a=config_.axes[j];
      if(!std::isfinite(p.positions[j]) || !std::isfinite(p.velocities[j]) || !std::isfinite(p.accelerations[j]) ||
          p.positions[j]<a.lower-1e-9 || p.positions[j]>a.upper+1e-9 || std::abs(p.velocities[j])>a.max_velocity+1e-6 ||
          std::abs(p.accelerations[j])>a.max_acceleration+1e-6 || (previous<0 && p.velocities[j]!=0)) return false;
    }
    previous=p.seconds;
  }
  if(tcp_origin_!=Time{} && previous-seconds(now-tcp_origin_)>kTcpMaxHorizon) return false;
  tcp_points_.insert(tcp_points_.end(),points.begin(),points.end());tcp_closed_=closed;return true;
}
ProgramPoint CanBusManager::tcp_reference(double t) const {
  if(t<=tcp_points_.front().seconds) return tcp_points_.front();
  if(t>=tcp_points_.back().seconds) return tcp_points_.back();
  std::size_t k=1;while(tcp_points_[k].seconds<t) ++k;
  return interpolate_points(tcp_points_[k-1],tcp_points_[k],t);
}
void CanBusManager::tcp_cut(double t,const std::string &reason,Time now) {
  if(tcp_points_.empty()) {
    // Ucięcie przed pierwszym punktem: celem jest pozycja zmierzona.
    ProgramPoint hold;
    for(unsigned j=0;j<kAxes;++j) {const auto f=streaming_feedback_position(j,now);hold.positions[j]=f.valid ? f.value : committed_positions_[j];}
    tcp_points_.push_back(hold);t=0;
  }
  t=std::clamp(t,tcp_points_.front().seconds,tcp_points_.back().seconds);
  auto end=tcp_reference(t);end.seconds=t;
  while(!tcp_points_.empty() && tcp_points_.back().seconds>=t-1e-9) tcp_points_.pop_back();
  tcp_points_.push_back(end);
  if(tcp_origin_==Time{}) tcp_origin_=now;
  tcp_end_=t;tcp_target_=end.positions;tcp_reason_=reason;tcp_final_started_={};
  trace_note("tcp_cut",reason);
}
bool CanBusManager::cut_tcp_jog(const std::string &session,std::uint64_t generation,const std::string &reason,Time now) {
  if(!active_ || source_!=Source::tcp_jog || session!=tcp_owner_ || generation!=control_generation_) return false;
  if(tcp_end_>=0) return true;
  if(tcp_origin_==Time{}) {tcp_cut(0,reason,now);return true;}
  // Bez rampy: koniec w punkcie, do którego napęd już jedzie (ostatni cel F5 = q(t + lookahead)).
  double lookahead=0;for(const auto &a:config_.axes) lookahead=std::max(lookahead,a.streaming.lookahead_time_sec);
  tcp_cut(seconds(now-tcp_origin_)+lookahead,reason,now);return true;
}
TcpTrajectoryState CanBusManager::tcp_trajectory(Time now) const {
  TcpTrajectoryState out;out.started=tcp_origin_!=Time{};out.elapsed=out.started ? std::max(0.,seconds(now-tcp_origin_)) : 0;
  out.buffered_until=tcp_points_.empty() ? 0 : tcp_points_.back().seconds;out.cut=tcp_end_>=0;out.end=tcp_end_;
  out.closed=tcp_closed_;out.done=tcp_done_;out.reason=tcp_reason_;return out;
}
bool CanBusManager::finish_tcp_jog(const std::string &session,std::uint64_t generation) {
  if(!active_ || source_!=Source::tcp_jog || session!=tcp_owner_ || generation!=control_generation_ || !tcp_done_) return false;
  active_=false;source_=Source::none;tcp_owner_.clear();tcp_last_stop_=last_tick_;tcp_points_.clear();
  message_="JOG TCP: zatrzymanie potwierdzone";return true;
}
// JOG TCP jako LIN o otwartym końcu: referencja q/v/a z kolejki w chwili slotu, F5 z tego samego
// streaming_setpoint co PTP/LIN (lookahead z v/a, korekta Kp ze znakiem, dithering RPM).
void CanBusManager::schedule_tcp_trajectory(Time now) {
  // Podtrzymanie panelu dotyczy otwartej kolejki; po ucięciu ruch kończy się bez panelu.
  if(tcp_end_<0 && (now<tcp_lease_ || now-tcp_lease_>=200ms)) {fault("JOG TCP: brak podtrzymania KEEPALIVE; wiek "+diagnostic_number(std::chrono::duration<double,std::milli>(now-tcp_lease_).count())+" ms, limit < 200 ms (cały robot)");return;}
  if(!feedback_good(now)) {fault("Zatrzymanie sterowania: "+feedback_failure_reason(now));return;}
  if(tcp_done_) return;
  const auto slot=static_cast<std::uint64_t>((now-tcp_started_)/kStreamingCycle);
  if(now<tcp_started_ || slot<tcp_slot_ || (now-tcp_started_)%kStreamingCycle>=kTcpSendWindow) return;
  // MKS odpowiada tym samym CAN ID co host: F5 nie wychodzi, dopóki osie robota czekają na odpowiedź.
  if(std::any_of(pending_.begin(),pending_.end(),[this](const Pending &p){return p.used && axis_index(p.address)<kAxes;})) return;
  const double cost=kAxes*can_frame_bits(8),budget=config_.bitrate*std::chrono::duration<double>(config_.cycle).count()*std::min(kPlannedUtilization,config_.utilization_limit);
  if(spent_bits_+cost>budget) {++stats_.deferred;return;}
  if(tcp_origin_==Time{}) {
    if(tcp_points_.empty() || (!tcp_closed_ && tcp_points_.back().seconds<kTcpStartBuffer)) return;
    tcp_origin_=now;
  }
  const double elapsed=seconds(now-tcp_origin_);
  if(tcp_end_<0 && elapsed>=tcp_points_.back().seconds)
    tcp_cut(tcp_points_.back().seconds,tcp_closed_ ? "Koniec drogi JOG TCP" : "Brak danych generatora TCP — kolejka zakończona",now);
  tcp_slot_=slot+1;
  if(elapsed>=tcp_end_ && tcp_end_>=0) {tcp_final(now,cost);return;}
  while(tcp_points_.size()>2 && tcp_points_[1].seconds<=elapsed) tcp_points_.pop_front();
  const auto point=tcp_reference(elapsed);
  const double remaining=std::max(0.,(tcp_end_>=0 ? tcp_end_ : tcp_points_.back().seconds)-elapsed);
  auto next_streaming=streaming_states_;
  for(unsigned j=0;j<kAxes;++j) {
    const auto &axis=config_.axes[j];const auto label=axis_diagnostic_label(j+1,axis.address);
    const auto feedback=streaming_feedback_position(j,now);
    if(feedback.valid) {
      if(std::abs(point.positions[j]-feedback.value)>1.25*kPi/180) {
        fault(label+": uchyb śledzenia JOG TCP="+diagnostic_number((point.positions[j]-feedback.value)*180/kPi)+"°, limit |uchyb| ≤ 1.25°; referencja="+diagnostic_number(point.positions[j]*180/kPi)+"°, pomiar="+diagnostic_number(feedback.value*180/kPi)+"°");return;
      }
      const double quantization=kPi/(axis.streaming.encoder_cpr*axis.gear_ratio);
      if(feedback.value<axis.lower-quantization-1e-9 || feedback.value>axis.upper+quantization+1e-9) {
        fault(label+": pomiar TCP="+diagnostic_number(feedback.value*180/kPi)+"° poza limitem mechanicznym ["+diagnostic_number(axis.lower*180/kPi)+", "+diagnostic_number(axis.upper*180/kPi)+"]°");return;
      }
    }
    try {
      const auto sample=streaming_setpoint(axis,point.positions[j],point.velocities[j],next_streaming[j],
          feedback.valid ? std::optional<double>(feedback.value) : std::nullopt,devices_[axis.address].microstep.value,
          point.accelerations[j],remaining,feedback.valid ? seconds(now-feedback.stamp) : 0.);
      std::copy(sample.bytes.begin(),sample.bytes.end(),stream_payloads_[j].begin());
    } catch(const std::exception &e) {fault(label+": JOG TCP — "+e.what());return;}
  }
  spent_bits_+=cost;
  for(unsigned j=0;j<kAxes;++j) {send_stream_payload(j,stream_payloads_[j],now);record_shaft_reference(j,point.positions[j],now);}
  streaming_states_=next_streaming;committed_positions_=point.positions;previous_velocity_=point.velocities;last_command_=now;
  motion_this_cycle_=true;++stats_.committed;
}
// Jak dolot końcowy PTP (final_approach + check_program_positions): po kFinalApproachDelay co kFinalApproachPeriod
// rozkaz F5 += cel − pomiar dla osi poza tolerancją; koniec po dwóch pomiarach w tolerancji odległych ≥ 100 ms.
void CanBusManager::tcp_final(Time now,double cost) {
  if(tcp_final_started_==Time{}) {
    tcp_final_started_=now;tcp_final_next_=now+kFinalApproachDelay;tcp_final_command_=tcp_target_;tcp_within_since_={};tcp_check_started_=now;
    previous_velocity_.fill(0);committed_positions_=tcp_target_;
  }
  bool fresh=true,within=true;std::string axes;
  for(unsigned j=0;j<kAxes;++j) {
    record_shaft_reference(j,tcp_target_[j],now);
    const auto &a=config_.axes[j];const auto &d=devices_[a.address];
    if(!d.position.fresh(now,kProgramFeedbackTimeout) || d.position.stamp<=tcp_check_started_) {fresh=false;continue;}
    const double error=tcp_target_[j]-position_to_joint(a,d.position.value);
    if(std::abs(error)>config_.program_position_tolerance[j]) {within=false;axes+=" J"+std::to_string(j+1)+"="+diagnostic_number(error*180/kPi)+"°";}
  }
  if(fresh && within) {
    if(tcp_within_since_==Time{}) {tcp_within_since_=now;tcp_check_started_=now;}
    else if(now-tcp_within_since_>=100ms) {tcp_done_=true;message_="JOG TCP: pozycja końcowa potwierdzona";}
    return;
  }
  if(fresh) tcp_within_since_={};
  if(now-tcp_final_started_>=kFinalApproachTimeout) {
    fault("JOG TCP: dolot końcowy nieudany po "+std::to_string(kFinalApproachTimeout.count())+" ms; uchyb (cel − pomiar):"+axes+
        ", tolerancja ±"+diagnostic_number(config_.program_position_tolerance[0]*180/kPi)+"°");return;
  }
  if(!fresh || within || now<tcp_final_next_) return;
  for(unsigned j=0;j<kAxes;++j) {
    const auto &axis=config_.axes[j];const auto &d=devices_[axis.address];
    const double measured=position_to_joint(axis,d.position.value),error=tcp_target_[j]-measured;
    if(std::abs(error)>config_.program_position_tolerance[j]) tcp_final_command_[j]=std::clamp(tcp_final_command_[j]+error,axis.lower,axis.upper);
    try {
      const auto sample=streaming_setpoint(axis,tcp_target_[j],0.,streaming_states_[j],measured,d.microstep.value,0.,0.,
          seconds(now-d.position.stamp),tcp_final_command_[j]);
      std::copy(sample.bytes.begin(),sample.bytes.end(),stream_payloads_[j].begin());
    } catch(const std::exception &e) {fault(axis_diagnostic_label(j+1,axis.address)+": dolot JOG TCP — "+e.what());return;}
  }
  spent_bits_+=cost;
  for(unsigned j=0;j<kAxes;++j) send_stream_payload(j,stream_payloads_[j],now);
  last_command_=now;tcp_final_next_=now+kFinalApproachPeriod;motion_this_cycle_=true;
  trace_note("tcp_final_approach",axes);
}
}
