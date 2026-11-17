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
// Uchyb śledzenia (referencja w chwili pomiaru − pomiar, jak SYNC): powyżej progu łagodne zatrzymanie (ucięcie kolejki,
// dolot, ENABLE bez zmian); twardy limit awarii = kRunawayTrackingLimitRad (4°). PTP normalnie ≤ 1,2°; JOG TCP przy
// szybkich J4/J6 dochodził do 2,0–2,07° (sesja 11.11.2026 21:09: 3 przerwania przy progu 2°) — próg 3°.
constexpr double kTcpTrackingSoftRad=3.*kPi/180;
// Historia referencji wstecz [s]: pomiar używany do porównania ma najwyżej kProgramFeedbackTimeout (250 ms).
constexpr double kTcpReferenceHistory=.3;
// Dolot: oś stoi, gdy pomiary przez to okno mieszczą się w ±tolerancji pozycji; korekta tylko dla osi stojącej
// (korekta w trakcie drgań rozbujała J3 w PTP 11.11.2026) albo gdy oś nie ustała po kTcpCorrectionWhileMoving.
constexpr auto kTcpStableWindow=std::chrono::milliseconds(150);
constexpr auto kTcpCorrectionWhileMoving=std::chrono::milliseconds(600);
// Postój bez F1=1: MKS bywa w F1=4 przy stojącym wale (11.11.2026); pozycja stała przez to okno też oznacza postój.
constexpr auto kTcpStandstillWithoutF1=std::chrono::milliseconds(500);
double seconds(Clock::duration d) {return std::chrono::duration<double>(d).count();}
double max_lookahead(const BusConfig &config) {
  double lookahead=0;for(const auto &a:config.axes) lookahead=std::max(lookahead,a.streaming.lookahead_time_sec);return lookahead;
}
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
  // Przejęcie dolotu tej samej sesji panelu (nowy ruch zaraz po puszczeniu drążka, bez czekania ~2 s na postój).
  // Atomowo pod blokadą właściciela: przy nieudanym starcie dolot trwa dalej, nadzór postoju nie widzi przerwy.
  const bool preempt=active_ && !fault_ && source_==Source::tcp_jog && session==tcp_owner_ && tcp_end_>=0 && tcp_final_started_!=Time{};
  if(preempt) {
    active_=false;source_=Source::none;tcp_owner_.clear();
    const auto generation=begin_tcp_jog(session,revision,model,sequence,issued,now);
    if(generation) {trace_note("tcp_final_preempted","nowy JOG TCP przejął dolot");return generation;}
    if(!active_) {active_=true;source_=Source::tcp_jog;tcp_owner_=session;}
    return 0;
  }
  if(!tcp_jog_available(now) || session.empty() || session.size()>64 || !sequence || issued>now ||
      now-issued>=std::chrono::milliseconds(100) || issued<=tcp_last_stop_ || revision!=profile_.revision ||
      model.empty() || model!=motion_model_ || config_.bitrate!=1000000 ||
      streaming_bus_utilization(6,config_.bitrate)>std::min(kPlannedUtilization,config_.utilization_limit)+1e-12 || !tcp_owner_.empty()) return 0;
  for(unsigned j=0;j<6;++j) if(!profile_.axes[j].configured || !manual_->calibration_valid(j+1) ||
      !devices_[config_.axes[j].address].mode.valid || devices_[config_.axes[j].address].mode.value!=5) return 0;
  if(program_owned_ && !active_ && !executor_.status.busy && executor_.status.state!="ready") program_owned_=false;
  if(!activate(Source::tcp_jog,now)) return 0;
  // Te same parametry F5 co PTP/LIN (lookahead, FF, Kp, dithering, acc).
  for(unsigned j=0;j<6;++j) config_.axes[j].streaming=program_streaming(j);
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
  tcp_cut(seconds(now-tcp_origin_)+max_lookahead(config_),reason,now);return true;
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
  // Podtrzymanie panelu dotyczy otwartej kolejki; po ucięciu ruch kończy się bez panelu. Utrata łączności z panelem
  // (zwykle ucina już TcpJogApi po 150 ms) = łagodne zatrzymanie jak puszczenie drążka, bez awarii i bez ENABLE=0.
  if(tcp_end_<0 && (now<tcp_lease_ || now-tcp_lease_>=200ms)) {
    const auto reason="Brak podtrzymania KEEPALIVE "+diagnostic_number(std::chrono::duration<double,std::milli>(now-tcp_lease_).count())+
        " ms (limit 200 ms) — łagodne zatrzymanie";
    trace_note("tcp_keepalive_lost",reason);
    tcp_cut(tcp_origin_==Time{} ? 0. : seconds(now-tcp_origin_)+max_lookahead(config_),reason,now);
  }
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
  // Historia referencji zostaje kTcpReferenceHistory wstecz: uchyb liczony dla chwili pomiaru.
  while(tcp_points_.size()>2 && tcp_points_[1].seconds<=elapsed-kTcpReferenceHistory) tcp_points_.pop_front();
  const auto point=tcp_reference(elapsed);
  const double remaining=std::max(0.,(tcp_end_>=0 ? tcp_end_ : tcp_points_.back().seconds)-elapsed);
  auto next_streaming=streaming_states_;std::string soft_stop;
  for(unsigned j=0;j<kAxes;++j) {
    const auto &axis=config_.axes[j];const auto label=axis_diagnostic_label(j+1,axis.address);
    const auto feedback=streaming_feedback_position(j,now);
    if(feedback.valid) {
      // Referencja w chwili odbioru pomiaru (jak SYNC): wiek pomiaru przy szybkim ruchu nie jest uchybem.
      const double measured_at=std::clamp(seconds(feedback.stamp-tcp_origin_),0.,elapsed);
      const double reference=tcp_reference(measured_at).positions[j],error=reference-feedback.value;
      const auto detail="uchyb śledzenia JOG TCP="+diagnostic_number(error*180/kPi)+"° (referencja w chwili pomiaru="+
          diagnostic_number(reference*180/kPi)+"°, pomiar="+diagnostic_number(feedback.value*180/kPi)+"°)";
      if(std::abs(error)>kRunawayTrackingLimitRad) {fault(label+": "+detail+" > 4° — ucieczka osi");return;}
      if(std::abs(error)>kTcpTrackingSoftRad && tcp_end_<0 && soft_stop.empty())
        soft_stop=label+": "+detail+" > "+diagnostic_number(kTcpTrackingSoftRad*180/kPi)+"° — łagodne zatrzymanie";
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
  if(!soft_stop.empty()) {trace_note("tcp_soft_stop",soft_stop);tcp_cut(elapsed+max_lookahead(config_),soft_stop,now);}
}
// Dolot końcowy (jak PTP final_approach + potwierdzenie postoju), odporny na drgania i przesunięcie układu celu F5
// napędu względem enkodera 0x31 (docs/motion_hardware_results.md 08.11 i 11.11.2026):
// - korekta (rozkaz F5 += cel − pomiar) co ≥ kFinalApproachPeriod, tylko dla osi poza tolerancją, która stoi
//   (pomiary w ±tolerancji pozycji przez kTcpStableWindow) albo nie ustała po kTcpCorrectionWhileMoving;
// - przez cały dolot cel końcowy co slot 10 ms (strumień trzymania), korekty zmieniają ten cel;
// - koniec dopiero po potwierdzonym postoju: wszystkie osie w tolerancji, stabilne, |RPM| ≤ 1 i F1=1 (albo pozycja stała
//   ≥ 500 ms), dwa razy w odstępie ≥ 100 ms (pomiary po starcie okna) — wtedy nadzór postoju kotwiczy pozycję trzymania;
// - awaria krytyczna tylko przy ucieczce: |cel − pomiar| > 4° albo uchyb rośnie po dwóch kolejnych korektach;
// - po kFinalApproachTimeout: pozycje wszystkich osi stałe → koniec z ostrzeżeniem (bez awarii); oś w ruchu → awaria.
void CanBusManager::tcp_final(Time now,double cost) {
  if(tcp_final_started_==Time{}) {
    tcp_final_started_=now;tcp_final_next_=now+kFinalApproachDelay;tcp_final_command_=tcp_target_;tcp_within_since_={};tcp_check_started_=now;
    previous_velocity_.fill(0);committed_positions_=tcp_target_;final_prev_error_.fill(-1);final_growth_.fill(0);
    final_stable_since_.fill(Time{});final_stable_seen_.fill(Time{});
  }
  bool fresh=true,newer=true,within=true,still=true,all_stable=true;std::string axes,moving;
  std::array<double,kAxes> error{};std::array<bool,kAxes> stable{};
  for(unsigned j=0;j<kAxes;++j) {
    record_shaft_reference(j,tcp_target_[j],now);
    const auto &a=config_.axes[j];const auto &d=devices_[a.address];const auto name=" J"+std::to_string(j+1);
    if(!d.position.fresh(now,kProgramFeedbackTimeout)) {fresh=newer=still=all_stable=false;moving+=name+": brak świeżej pozycji;";continue;}
    if(d.position.stamp>final_stable_seen_[j]) {
      // Okno = tolerancja pozycji (min. kHoldMinimumTicks): J4 w trzymaniu oscyluje ±55 imp. wału (0,24° przegubu, 11.11.2026).
      const double gear=std::isfinite(a.gear_ratio) && a.gear_ratio>0 ? a.gear_ratio : 1.;
      const double band=std::max(kHoldMinimumTicks,config_.program_position_tolerance[j]*gear*16384/(2*kPi));
      if(final_stable_since_[j]==Time{} || std::abs(static_cast<double>(d.position.value-final_stable_anchor_[j]))>band) {
        final_stable_anchor_[j]=d.position.value;final_stable_since_[j]=d.position.stamp;
      }
      final_stable_seen_[j]=d.position.stamp;
    }
    stable[j]=d.position.stamp-final_stable_since_[j]>=kTcpStableWindow;
    error[j]=tcp_target_[j]-position_to_joint(a,d.position.value);
    if(std::abs(error[j])>kRunawayTrackingLimitRad) {
      fault(axis_diagnostic_label(j+1,a.address)+": dolot JOG TCP — oś "+diagnostic_number(error[j]*180/kPi)+
          "° od celu (cel − pomiar), limit 4° — ucieczka osi");return;
    }
    if(std::abs(error[j])>config_.program_position_tolerance[j]) {within=false;axes+=name+"="+diagnostic_number(error[j]*180/kPi)+"°";}
    const bool settled_long=d.position.stamp-final_stable_since_[j]>=kTcpStandstillWithoutF1;
    if(!stable[j]) all_stable=false;
    const bool standing=stable[j] && d.speed.fresh(now,150ms) && std::abs(d.speed.value)<=kHoldingSpeedNoiseRpm &&
        ((d.motor_state.fresh(now,150ms) && d.motor_state.value==1) || settled_long);
    if(!standing) {
      still=false;
      moving+=name+": RPM="+(d.speed.valid ? std::to_string(d.speed.value) : std::string("?"))+", F1="+
          (d.motor_state.valid ? std::to_string(d.motor_state.value) : std::string("?"))+(stable[j] ? "" : ", pozycja zmienna")+";";
    }
    if(d.position.stamp<=tcp_check_started_ || d.speed.stamp<=tcp_check_started_ || d.motor_state.stamp<=tcp_check_started_) newer=false;
  }
  const bool settled=fresh && within && still;
  if(tcp_within_since_!=Time{}) {
    // Potwierdzenie z pomiarów po starcie okna; drgnięcie albo wyjście z tolerancji = od nowa.
    if(newer && settled && now-tcp_within_since_>=100ms) {tcp_done_=true;message_="JOG TCP: pozycja końcowa i postój potwierdzone";return;}
    if(newer && !settled) tcp_within_since_={};
  } else if(settled) {tcp_within_since_=now;tcp_check_started_=now;}
  if(now-tcp_final_started_>=kFinalApproachTimeout) {
    if(fresh && all_stable) {
      // Wały stoją, ale korekty nie sprowadziły osi do tolerancji albo napęd nie zgłasza F1=1: koniec z ostrzeżeniem.
      tcp_done_=true;
      if(!within) tcp_reason_+="; dolot niepełny po "+std::to_string(kFinalApproachTimeout.count())+" ms (cel − pomiar):"+axes;
      message_="JOG TCP: postój potwierdzony pozycją"+(within ? std::string() : ", dolot niepełny:"+axes);
      trace_note("tcp_final_incomplete",within ? moving : axes);return;
    }
    fault("JOG TCP: brak postoju "+std::to_string(kFinalApproachTimeout.count())+" ms po końcu ruchu (wymagana stała pozycja, |RPM| ≤ "+
        std::to_string(kHoldingSpeedNoiseRpm)+"):"+moving+(axes.empty() ? "" : " uchyb:"+axes));return;
  }
  if(!fresh) return;
  bool correct=false;
  if(!within && now>=tcp_final_next_) for(unsigned j=0;j<kAxes;++j) {
    const auto &axis=config_.axes[j];
    if(std::abs(error[j])<=config_.program_position_tolerance[j]) continue;
    if(!stable[j] && now-tcp_final_next_<kTcpCorrectionWhileMoving-kFinalApproachPeriod) continue;
    // Uchyb rosnący w kolejnych korektach (oś oddala się mimo korekty celu) = ucieczka, jak w SYNC.
    const double magnitude=std::abs(error[j]);
    final_growth_[j]=final_prev_error_[j]>=0 && magnitude>final_prev_error_[j]+kRunawayFinalGrowthRad ? final_growth_[j]+1 : 0;
    final_prev_error_[j]=magnitude;
    if(final_growth_[j]>=2 && magnitude>kRunawayFinalMinRad) {
      fault(axis_diagnostic_label(j+1,axis.address)+": ucieczka osi w dolocie JOG TCP — uchyb rośnie mimo korekt ("+
          diagnostic_number(magnitude*180/kPi)+"°)");return;
    }
    tcp_final_command_[j]=std::clamp(tcp_final_command_[j]+error[j],axis.lower,axis.upper);correct=true;
  }
  // Strumień trzymania: cel końcowy (z korektami) w każdym slocie 10 ms do potwierdzenia postoju. Bez niego MKS po
  // dojechaniu do ostatniego celu potrafił ruszyć z powrotem w stronę wcześniejszych celów trasy (11.11.2026).
  for(unsigned j=0;j<kAxes;++j) {
    const auto &axis=config_.axes[j];const auto &d=devices_[axis.address];
    const double measured=position_to_joint(axis,d.position.value);
    try {
      const auto sample=streaming_setpoint(axis,tcp_target_[j],0.,streaming_states_[j],measured,d.microstep.value,0.,0.,
          seconds(now-d.position.stamp),tcp_final_command_[j]);
      std::copy(sample.bytes.begin(),sample.bytes.end(),stream_payloads_[j].begin());
    } catch(const std::exception &e) {fault(axis_diagnostic_label(j+1,axis.address)+": dolot JOG TCP — "+e.what());return;}
  }
  spent_bits_+=cost;
  for(unsigned j=0;j<kAxes;++j) send_stream_payload(j,stream_payloads_[j],now);
  last_command_=now;motion_this_cycle_=true;
  if(correct) {tcp_final_next_=now+kFinalApproachPeriod;trace_note("tcp_final_approach",axes);}
}
}
