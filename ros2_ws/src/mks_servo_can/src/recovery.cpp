#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/diagnostic_message.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include "mks_servo_can/trace.hpp"
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;
namespace mks_servo_can {
namespace {
constexpr auto kRecoveryTimeout=5s;
constexpr auto kStopPhaseGap=6ms; // Próby na 1.0.9: 5 ms bez błędów CAN +20%.
constexpr std::int64_t kRestTicks=22; // floor(0,5° * 16384 / 360); nigdy >0,5°.
const char *phase_name(unsigned phase) {
  static constexpr const char *names[]={"idle","stopping","verifying_disabled","overwriting",
      "restoring","verifying","idle_disabled","recovery_failed"};
  return phase<8 ? names[phase] : "recovery_failed";
}
}
std::string CanBusManager::recovery_state_name() const {
  return phase_name(static_cast<unsigned>(recovery_.phase));
}

void CanBusManager::begin_recovery(const std::string &reason,bool force_disable,Time now) {
  last_tick_=now;
  if(recovery_.phase==RecoveryPhase::stopping && stats_.stop_pending) {
    // Wielokrotny STOP nie przesuwa wysyłki ENABLE=0 ani nie dokłada
    // kolidujących serii. Żądanie operatora może zaostrzyć politykę awarii.
    recovery_.force_disable=recovery_.force_disable || force_disable;
    return;
  }
  unverified_sync_buffer_=unverified_sync_buffer_ || executor_.sync_uncertain || recovery_.staged_buffer_seen ||
      (source_==Source::program && sync_recovery_required_);
  ++control_generation_;++program_generation_;latest_.reset();active_=false;source_=Source::none;
  tcp_owner_.clear();tcp_braking_=false;tcp_last_stop_=now;lease_stamp_={};program_owned_=false;scanning_=false;
  executor_.staged=false;executor_.sync_waiting_confirmation=false;
  if(executor_.status.busy || executor_.status.state=="ready") {
    executor_.status.busy=false;executor_.status.state="failed";executor_.status.message=reason;
  }
  for(auto &p:pending_) p.used=false;
  sync_recovery_required_=true;fault_=true;stop_sent_=true;
  recovery_={};recovery_.phase=RecoveryPhase::stopping;recovery_.message="STOP wysłany";
  recovery_.staged_buffer_seen=unverified_sync_buffer_;recovery_.verify_after=now;
  last_fault_diagnostic_cycle_=std::numeric_limits<std::uint64_t>::max();
  fault_diagnostic_cursor_=0;
  recovery_.force_disable=force_disable;recovery_.started=now;recovery_.deadline=now+kRecoveryTimeout;
  stats_.stop_failed=0;stats_.stop_pending=true;stats_.stop_f7_acks=stats_.stop_zero_acks=0;
  for(unsigned j=0;j<kAxes;++j) {
    const auto address=config_.axes[j].address ? config_.axes[j].address : devices_[j+1].detected ? j+1 : 0;
    recovery_.addresses[j]=address;if(!address) continue;
    recovery_.required|=1u<<j;recovery_.pending|=1u<<j;
    next_position_[address]=next_speed_[address]=next_enable_[address]=next_motor_[address]={};
  }
  manual_->transport_fault();message_=reason+"; STOP wysłany; trwa potwierdzanie ENABLE i postoju";
  send_recovery_stop_phase(0,now);
  trace_note("recovery_started",reason,0,force_disable);
}

void CanBusManager::send_recovery_stop_phase(unsigned phase,Time now) {
  for(unsigned j=0;j<kAxes;++j) {
    const auto address=recovery_.addresses[j];
    if(!address || (phase==2 && !recovery_.force_disable)) continue;
    const auto data=phase==0 ? stop_request(address) : phase==1 ? absolute_position_stop_request(address) : enable_request(address,false);
    try {
      if(!transport_->is_open()) throw std::runtime_error("transport CAN zamknięty");
      transport_->send(address,data);++stats_.tx;
    } catch(const std::exception &e) {
      stats_.stop_failed|=1u<<j;
      message_+="; "+axis_diagnostic_label(j+1,address)+": "+e.what();
    } catch(...) {stats_.stop_failed|=1u<<j;}
  }
  recovery_.stop_phase=phase+1;
  recovery_.stop_next=current_time(now)+kStopPhaseGap;
  stats_.stop_elapsed_ms=std::chrono::duration<double,std::milli>(current_time(now)-recovery_.started).count();
}

bool CanBusManager::send_recovery(std::uint16_t address,const std::vector<std::uint8_t> &data,
    std::size_t response_length,Time now) {
  if(!transport_->is_open() || !can_send(address,now) || !reserve(data.size(),response_length,true)) return false;
  transport_->send(address,data);++stats_.tx;
  for(auto &p:pending_) if(!p.used) {
    p={true,address,Kind::recovery,data[0]==0 ? data[1] : data[0],response_length,now,now+120ms};
    recovery_.waiting=true;return true;
  }
  return false;
}

bool CanBusManager::recovery_standstill(Time now,bool capture) {
  bool complete=true;unsigned pending=0;
  for(unsigned j=0;j<kAxes;++j) if(recovery_.required&(1u<<j)) {
    const auto address=recovery_.addresses[j];const auto &d=devices_[address];auto &anchor=recovery_.anchors[j];
    const bool base=d.enable.fresh(now,300ms) && !d.enable.value && d.speed.fresh(now,150ms) && d.speed.value==0 &&
        d.motor_state.fresh(now,200ms) && d.motor_state.value==1 && d.position.fresh(now,250ms) &&
        d.enable.stamp>=recovery_.verify_after && d.speed.stamp>=recovery_.verify_after &&
        d.motor_state.stamp>=recovery_.verify_after && d.position.stamp>=recovery_.verify_after;
    if(!base) {complete=false;pending|=1u<<j;anchor.valid=false;continue;}
    if(!anchor.valid || std::abs(d.position.value-anchor.value)>kRestTicks) {anchor=d.position;complete=false;pending|=1u<<j;continue;}
    if(d.position.stamp<anchor.stamp+100ms) {complete=false;pending|=1u<<j;}
  }
  recovery_.pending=pending;
  if(capture && complete) trace_note("recovery_standstill_confirmed","dwa zestawy >=100 ms; tolerancja 0,5 stopnia");
  return complete;
}

void CanBusManager::recovery_failed(const std::string &reason) {
  std::string detail=reason,unknown_watchdog;
  const bool unconfirmed_stop=recovery_.phase==RecoveryPhase::stopping || recovery_.phase==RecoveryPhase::verifying_disabled;
  recovery_.phase=RecoveryPhase::failed;recovery_.waiting=false;
  stats_.stop_pending=false;
  for(auto &p:pending_) if(p.kind==Kind::recovery) p.used=false;
  for(unsigned j=0;j<kAxes;++j) if(recovery_.pending&(1u<<j)) {
    if(!unconfirmed_stop) continue;
    const auto address=recovery_.addresses[j];const auto &w=devices_[address].watchdog_ms;
    if(w.fresh(last_tick_,2s) && w.value) {
      const auto resume=last_tick_+std::chrono::milliseconds(w.value)+50ms;
      recovery_.watchdog_pause_until[j]=resume;
      quarantine_[address]=resume;
    } else unknown_watchdog+=(unknown_watchdog.empty() ? "" : ", ")+axis_diagnostic_label(j+1,address);
  }
  if(!unknown_watchdog.empty()) detail+="; brak świeżego niezerowego watchdogu: "+unknown_watchdog+
      " — nie przyjęto założenia o zatrzymaniu";
  recovery_.message=detail;message_="Odzyskiwanie nieudane: "+detail;
  trace_note("recovery_failed",detail);
}

void CanBusManager::recovery_reply(const Frame &frame,Time stamp) {
  (void)stamp;bool good=false;auto &r=recovery_;auto &device=devices_[frame.address];
  if(!r.waiting || r.axis>=kAxes || frame.address!=r.addresses[r.axis]) {recovery_failed("nieoczekiwana odpowiedź odzyskiwania");return;}
  if(r.phase==RecoveryPhase::overwriting) {
    if(r.step==0) good=frame.data[1]==1;
    else if(r.step==1) {
      good=frame.data[1]==1 && frame.data[2]==0;
      if(good) {device.responses_known=true;device.responses_disabled=false;}
    }
    else if(r.step==2) {
      good=frame.data[1]<=1;
      if(good) {device.synchronization_known=true;device.synchronization_disabled=frame.data[1]==0;}
      if(good && frame.data[1]==1) {
        r.staged_buffer_seen=unverified_sync_buffer_=true;
        const auto &identity=devices_[frame.address].identity;
        if(identity.model!=Model::kServo42D || identity.firmware_major!=1 || identity.firmware_minor!=0 || identity.firmware_patch!=9) {
          recovery_failed(axis_diagnostic_label(r.axis+1,frame.address)+": nadpisanie SYNC wymaga MKS SERVO42D CAN firmware 1.0.9");return;
        }
      }
      if(good && frame.data[1]==0) {r.pending&=~(1u<<r.axis);++r.axis;r.step=0;r.waiting=false;return;}
    } else if(r.step==3) good=frame.data[1]==5;
  } else if(r.phase==RecoveryPhase::restoring) {
    if(r.step==0 || r.step==2) good=frame.data[1]==1;
    else if(r.step==1) {
      good=frame.data[1]==0;if(good) {device.synchronization_known=true;device.synchronization_disabled=true;}
    } else if(r.step==3) {
      good=frame.data[1]==0 && frame.data[2]==0;if(good) {device.responses_known=true;device.responses_disabled=true;}
    }
  }
  if(!good) {
    recovery_failed(axis_diagnostic_label(r.axis+1,frame.address)+": odrzucony krok odzyskiwania; opcode="+
        std::to_string(frame.data[0])+", status="+std::to_string(frame.data[1]));return;
  }
  r.waiting=false;
  if(r.phase==RecoveryPhase::overwriting && r.step==3) {
    r.pending&=~(1u<<r.axis);++r.axis;r.step=0;
  } else if(r.phase==RecoveryPhase::restoring && r.step==3) {
    r.pending&=~(1u<<r.axis);++r.axis;r.step=0;
  }
  else ++r.step;
}

void CanBusManager::schedule_fault_diagnostics(Time now) {
  if(!transport_->is_open() || recovery_.phase==RecoveryPhase::stopping || last_fault_diagnostic_cycle_==cycle_number_) return;
  last_fault_diagnostic_cycle_=cycle_number_;
  static constexpr Kind reads[]={Kind::identity,Kind::position,Kind::speed,Kind::enable,
      Kind::stall,Kind::motor,Kind::sync_mode,Kind::watchdog};
  // Cztery niezależne zapytania na cykl mieszczą się w istniejącej puli pending
  // i budżecie 40%. Pełny obieg 8 odczytów x 6 osi trwa ok. 120 ms, dzięki
  // czemu diagnostyka pozostaje świeża także przy zatrzaśniętej awarii ruchu.
  unsigned sent=0,scanned=0;
  // Nie zawijamy całego obiegu w jednym ticku: przy mniej niż czterech
  // wykrytych osiach wrócilibyśmy do tego samego rodzaju odczytu co cykl.
  while(sent<4 && scanned<kAxes) {
    const auto slot=fault_diagnostic_cursor_%(kAxes*8);const auto joint=slot%kAxes;
    if(recovery_.watchdog_pause_until[joint]>now || fault_probe_after_[joint]>now || !recovery_.addresses[joint]) {
      ++fault_diagnostic_cursor_;++scanned;continue;
    }
    const auto address=recovery_.addresses[joint];
    if(!can_send(address,now)) {
      const auto full=static_cast<std::size_t>(std::count_if(
          pending_.begin(),pending_.end(),[](const Pending &p){return p.used;}))==pending_.size();
      if(full) break;
      ++fault_diagnostic_cursor_;++scanned;continue;
    }
    // W awarii nie ma obowiązkowego streamingu, ale nadal obowiązuje całkowity
    // limit 40%. Odczyty odzyskiwania korzystają z tej puli jako krytyczne,
    // zamiast dodatkowo rezerwować pasmo dla nieistniejących komend ruchu.
    if(!send_query(address,reads[slot/kAxes],now,true)) break;
    ++fault_diagnostic_cursor_;++scanned;++sent;
  }
}

void CanBusManager::schedule_recovery(Time now) {
  auto &r=recovery_;
  if(r.phase==RecoveryPhase::idle || r.phase==RecoveryPhase::idle_disabled || r.phase==RecoveryPhase::failed) return;
  if(r.phase==RecoveryPhase::verifying_disabled && !r.force_disable) {
    bool operator_needed=false;
    for(unsigned j=0;j<kAxes;++j) if(r.required&(1u<<j)) {
      const auto &enable=devices_[r.addresses[j]].enable;
      operator_needed=operator_needed || (enable.fresh(now,300ms) && enable.value);
    }
    if(operator_needed) {
      r.deadline=now+kRecoveryTimeout;
      r.message="STOP wysłany; reguła ENABLE wyłączona — operator musi wyłączyć napędy";
    }
  }
  if(now>=r.deadline) {recovery_failed("przekroczono termin etapu "+std::string(phase_name(static_cast<unsigned>(r.phase))));return;}
  if(r.phase==RecoveryPhase::stopping) {
    if(now<r.stop_next) return;
    if(r.stop_phase<3) {send_recovery_stop_phase(r.stop_phase,now);return;}
    // Także po F3 zostawiamy okno na odpowiedzi napędów przed zapytaniami.
    stats_.stop_pending=false;r.phase=RecoveryPhase::verifying_disabled;
    r.message="Oczekiwanie na ENABLE=0 i postój";r.anchors={};r.verify_after=current_time(now);
    return;
  }
  if(!r.required) {recovery_failed("brak skonfigurowanych ani wykrytych osi do potwierdzenia STOP");return;}
  if(r.phase==RecoveryPhase::verifying_disabled) {
    if(!recovery_standstill(now,true)) return;
    r.phase=RecoveryPhase::overwriting;r.message="ENABLE wyłączone; nadpisywanie bufora SYNC";
    r.axis=r.step=0;r.pending=r.required;r.deadline=now+kRecoveryTimeout;return;
  }
  if(r.phase==RecoveryPhase::overwriting || r.phase==RecoveryPhase::restoring) {
    if(r.waiting) return;
    for(unsigned j=0;j<kAxes;++j) if(r.required&(1u<<j)) {
      const auto &device=devices_[r.addresses[j]];
      if(device.enable.fresh(now,300ms) && device.enable.value) {
        r.pending|=1u<<j;recovery_failed(axis_diagnostic_label(j+1,r.addresses[j])+": ENABLE ponownie włączone podczas odzyskiwania");return;
      }
      if(!device.enable.fresh(now,300ms) || !device.speed.fresh(now,150ms) || device.speed.value!=0) return;
    }
    while(r.axis<kAxes && !(r.required&(1u<<r.axis))) ++r.axis;
    if(r.axis==kAxes) {
      if(r.phase==RecoveryPhase::overwriting) {
        // Dopóki sekwencja nie ma odbioru sprzętowego, pozostawiamy 4A=1.
        // Dzięki temu restart procesu nie ukryje niepewnego bufora za 4A=0.
        if(r.staged_buffer_seen && !config_.program_staged_reset_verified) {
          r.pending=r.required;
          recovery_failed("wymagany odbiór sprzętowy usuwania oczekującego F5; zachowano SYNC=1");return;
        }
        r.phase=RecoveryPhase::restoring;r.message="Przywracanie 4A=0 i 8C=00/00";
        r.axis=r.step=0;r.pending=r.required;r.deadline=now+kRecoveryTimeout;return;
      }
      r.phase=RecoveryPhase::verifying;r.message="Postój potwierdzony; końcowa weryfikacja";
      r.anchors={};r.verify_after=now;r.deadline=now+kRecoveryTimeout;return;
    }
    const auto address=r.addresses[r.axis];std::vector<std::uint8_t> data;std::size_t length=3;
    if(r.phase==RecoveryPhase::overwriting) {
      if(r.step==0) data=request(address,0x8c,{1,0});
      else if(r.step==1) {data=parameter_request(address,0x8c);length=4;}
      else if(r.step==2) data=parameter_request(address,0x4a);
      else data=absolute_position_stop_request(address);
    } else {
      if(r.step==0) data=request(address,0x4a,{0});
      else if(r.step==1) data=parameter_request(address,0x4a);
      else if(r.step==2) data=request(address,0x8c,{0,0});
      else {data=parameter_request(address,0x8c);length=4;}
    }
    send_recovery(address,data,length,now);return;
  }
  if(r.phase==RecoveryPhase::verifying && recovery_standstill(now,true)) {
    if(r.staged_buffer_seen && !config_.program_staged_reset_verified) {
      recovery_failed("wymagany odbiór sprzętowy usuwania oczekującego F5");return;
    }
    r.phase=RecoveryPhase::idle_disabled;r.pending=0;r.message="Gotowy do ponownego załączenia";
    r.staged_buffer_seen=false;executor_.sync_uncertain=false;
    unverified_sync_buffer_=false;sync_recovery_required_=false;fault_=false;stop_sent_=false;message_=r.message;manual_->recovery_complete();
    trace_note("recovery_complete",r.message);
  }
}

void CanBusManager::reconnect(Time now) {
  if(transport_->is_open() || now<next_reconnect_) return;
  try {
    transport_->open(config_.interface);cycle_initialized_=false;
    if(transport_->bitrate() && transport_->bitrate()!=config_.bitrate)
      throw std::runtime_error("Bitrate konfiguracji różni się od fizycznego interfejsu CAN po reconnect");
    message_="Połączenie CAN przywrócone; sterowanie pozostaje zablokowane do zakończenia odzyskiwania";
    trace_note("socket_reconnected",config_.interface);
    if(fault_) begin_recovery("Połączenie CAN przywrócone po utracie transportu",recovery_.force_disable,now);
  } catch(const std::exception &e) {
    transport_lost(now,e.what());
  }
}

void CanBusManager::transport_lost(Time now,const std::string &reason) {
  static constexpr std::chrono::milliseconds delays[]={500ms,1000ms,2000ms,5000ms};
  try {transport_->close();} catch(...) {}
  stats_.stop_pending=false; // Po reconnect odtworzyć pełną sekwencję STOP.
  const auto index=std::min<unsigned>(reconnect_attempt_,3);
  next_reconnect_=now+delays[index];++reconnect_attempt_;
  message_="Brak połączenia CAN; ponowna próba za "+std::to_string(delays[index].count())+" ms: "+reason;
  trace_note("socket_transport_lost",message_);
}
} // namespace mks_servo_can
