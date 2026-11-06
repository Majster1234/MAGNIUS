#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/diagnostic_message.hpp"
#include "mks_servo_can/trace.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace mks_servo_can {
using namespace std::chrono_literals;
namespace {
constexpr auto kSyncLateness=2500us;
// Pojedynczy brak ACK nie przerywa ruchu: oś bez nowego F5 jedzie dalej do poprzedniego celu (~75 ms dalej
// na trajektorii). Awaria dopiero przy trzecim kolejnym cyklu bez ACK tej samej osi.
constexpr unsigned kSyncMissedAckLimit=2;
// Spóźnienie wyzwolenia lub potwierdzenia ponad 2,5 ms przesuwa oś czasu programu zamiast przerywać ruch:
// wszystkie osie wykonują ten sam zestaw później o tę samą wartość, tor się nie zmienia. Powyżej limitu — awaria.
constexpr auto kSyncMaxShift=10ms;
// Minimalny czas od wysłania zestawu do SYNC na odebranie ACK, gdy przygotowanie samo się spóźniło.
constexpr auto kSyncPrepareMargin=3ms;
double ms(Clock::duration d) {return std::chrono::duration<double,std::milli>(d).count();}
std::uint8_t participants(const MotionProgramStatus &s) {
  std::uint8_t mask=0;for(unsigned j=0;j<6;++j) if(s.participating[j]) mask|=1u<<j;return mask;
}
void sync_trace(const char *name,const MotionExecutor &e,Time now,unsigned axis=0,
    const std::vector<std::uint8_t> *data=nullptr) {
  TraceEvent t;t.event=name;t.mono_ns=trace_ns(now);t.axis=axis;
  t.deadline_ns=trace_ns(e.due);t.request_ns=trace_ns(e.batch_started);t.flags=e.staged_ack_mask;
  if(axis>=1 && axis<=6) {
    const auto j=axis-1;
    std::snprintf(t.detail.data(),t.detail.size(),"cycle=%zu phase=%.9f horizon=%.9f q=%.12g v=%.12g a=%.12g",
        e.frame,e.staged_reference.seconds,e.staged_horizon,e.staged_reference.positions[j],e.staged_reference.velocities[j],e.staged_reference.accelerations[j]);
  } else std::snprintf(t.detail.data(),t.detail.size(),"cycle=%zu phase=%.9f horizon=%.9f",e.frame,e.staged_reference.seconds,e.staged_horizon);
  if(data) {t.dlc=data->size();std::copy(data->begin(),data->end(),t.data.begin());t.command=data->front();}
  trace(t);
}
}
Time CanBusManager::next_deadline() const {
  const auto &e=executor_;
  if(!active_ || fault_ || source_!=Source::program || e.status.dispatch_mode!="synchronized" || !e.staged || e.sync_waiting_confirmation) return {};
  return e.due;
}
bool CanBusManager::sync_shift(Time due,Time now,const char *reason) {
  auto &e=executor_;const auto delta=due-e.due;
  if(delta<=Clock::duration::zero()) return true;
  if(delta>kSyncMaxShift) {
    fault(std::string("SYNC: ")+reason+"="+diagnostic_number(ms(delta))+" ms, limit przesunięcia "+diagnostic_number(ms(kSyncMaxShift))+
        " ms; cykl="+std::to_string(e.frame));return false;
  }
  e.phase_origin+=delta;e.due=due;if(e.started!=Time{}) e.started+=delta;
  ++stats_.sync_shifts;stats_.sync_shift_ms.add(ms(delta));
  sync_trace("sync_shifted",e,now);trace_note("sync_shifted",std::string(reason)+" "+diagnostic_number(ms(delta))+" ms");
  return true;
}
bool CanBusManager::sync_ack_outstanding(std::uint16_t address) const {
  const auto &e=executor_;const auto j=axis_index(address);
  if(!active_ || source_!=Source::program || e.status.dispatch_mode!="synchronized" || j>=kAxes ||
      !e.program || (e.frame>=e.program->frames.size() && !e.correction)) return false;
  return (((e.staged ? e.staged_sent_mask&~e.staged_ack_mask : 0)|e.late_ack_mask)>>j)&1u;
}
bool CanBusManager::synchronized_query_window(Time now) const {
  const auto &e=executor_;
  if(e.status.dispatch_mode!="synchronized") return true;
  if(e.status.state=="arming" || e.status.state=="restoring") return false;
  if(!e.program || e.frame>=e.program->frames.size()) return true;
  // Zestaw k+1 powstaje po echu SYNC k. Przerwa 4 ms przed najwcześniejszą
  // kolejną wysyłką F5; RX i potwierdzenia nie są wstrzymywane.
  return e.staged && !e.sync_waiting_confirmation && now<std::min(e.due,e.batch_started+kStreamingCycle)-4ms;
}
bool CanBusManager::synchronized_reply(const Frame &frame,Time stamp) {
  auto &e=executor_;auto &s=e.status;
  if(source_!=Source::program || s.dispatch_mode!="synchronized" || frame.data.empty() || frame.data[0]!=0xf5) return false;
  const auto j=axis_index(frame.address);
  if(j>=kAxes || !s.participating[j]) return false;
  if(!validate_response(frame.address,frame.data,0xf5,3).valid) {
    fault(axis_diagnostic_label(j+1,frame.address)+": nieprawidłowe ACK F5 w trybie SYNC");return true;
  }
  const std::uint8_t bit=1u<<j;
  // ACK przed wysłaniem SYNC jest w czasie: napęd ma komendę w buforze przed wyzwoleniem (także po Tk, gdy SYNC
  // jeszcze nie wyszedł, np. przy przesunięciu osi czasu).
  const bool current=frame.data[1]==5 && e.staged && !e.sync_waiting_confirmation && (e.staged_sent_mask&bit) &&
      stamp>=e.staged_sent[j] && !(e.staged_ack_mask&bit);
  if(!current) {
    // Spóźniony ACK zestawu, który wyzwolono bez niego. Ramka F5/05 nie ma numeru cyklu: pierwszą nadmiarową
    // odpowiedź osi z late_ack_mask uznajemy za tę spóźnioną (także gdy wcześniej zaliczono ją bieżącemu zestawowi).
    if(frame.data[1]==5 && (e.late_ack_mask&bit)) {e.late_ack_mask&=~bit;sync_trace("sync_ack_late",e,stamp,frame.address,&frame.data);return true;}
    fault(axis_diagnostic_label(j+1,frame.address)+": brak zgodnego przyjęcia F5 do bufora SYNC; status="+
        std::to_string(frame.data[1])+", cykl="+std::to_string(e.frame));return true;
  }
  stats_.ack_margin_ms.add(std::chrono::duration<double,std::milli>(e.due-stamp).count());
  e.staged_ack_mask|=1u<<j;s.sync_ack_mask=e.staged_ack_mask;
  sync_trace("sync_axis_accepted",e,stamp,frame.address,&frame.data);return true;
}
void CanBusManager::synchronized_confirmation(const Frame &frame,Time now) {
  auto &e=executor_;auto &s=e.status;
  if(!active_ || source_!=Source::program || s.dispatch_mode!="synchronized") return;
  if(frame.address!=0 || frame.data!=sync_payload_) return;
  const auto stamp=frame.received_at==Time{} ? now : frame.received_at;
  // Echo po > 2,5 ms: SYNC wyszedł na przewód później — oś czasu przesuwa się do chwili nadania.
  if(e.sync_waiting_confirmation && stamp>=e.sync_sent && stamp>e.due+kSyncLateness && !sync_shift(stamp,now,"spóźnione potwierdzenie nadania SYNC")) return;
  if(!e.sync_waiting_confirmation || stamp<e.sync_sent || stamp>e.due+kSyncLateness) {
    fault("SYNC: niezgodne lub spóźnione potwierdzenie nadania; cykl="+std::to_string(e.frame));return;
  }
  if(e.correction) {
    // Zestaw korekcyjny dolotu: bez zmiany numeru zestawu i referencji; nowa kontrola pozycji po wyzwoleniu.
    sync_trace("sync_correction_activated",e,stamp);
    for(unsigned j=0;j<kAxes;++j) if(s.participating[j] && (e.staged_ack_mask&(1u<<j))) record_shaft_command(j,stream_payloads_[j],stamp);
    streaming_states_=staged_streaming_states_;last_command_=stamp;
    e.correction=e.sync_waiting_confirmation=e.staged=false;e.last_sync=stamp;e.settled={};
    s.sync_state="activated";begin_program_check(now);return;
  }
  sync_trace("sync_activated",e,stamp);
  for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) {
    sync_trace("sync_reference_committed",e,stamp,config_.axes[j].address,&stream_payloads_[j]);
    // Wykresy wałów: referencja i cel F5/RPM w chwili wyzwolenia (wtedy napęd wykonuje zestaw); oś bez ACK zachowuje stary cel.
    record_shaft_reference(j,e.staged_reference.positions[j],stamp);
    if(e.staged_ack_mask&(1u<<j)) record_shaft_command(j,stream_payloads_[j],stamp);
  }
  streaming_states_=staged_streaming_states_;committed_positions_=e.staged_reference.positions;
  previous_velocity_=e.staged_reference.velocities;last_command_=e.due;
  if(e.started==Time{}) e.started=e.phase_origin;
  s.sync_confirmed_monotonic_ns=trace_ns(stamp);
  s.sync_lateness_ms=std::chrono::duration<double,std::milli>(stamp-e.due).count();
  stats_.sync_lateness_ms.add(s.sync_lateness_ms);
  s.sync_cycle=e.frame;s.sync_state="activated";
  s.elapsed=std::min(e.program->duration,e.frame*kStreamingCycleSeconds);
  s.progress=std::min(.99,s.elapsed/e.program->duration);
  ++e.frame;++stats_.committed;e.last_sync=stamp;
  e.sync_waiting_confirmation=e.staged=false;
  e.due=e.phase_origin+kStreamingCycle*e.frame;
  if(e.frame==e.program->frames.size()) {s.state="settling";e.next_check=now;begin_program_check(now);}
}
void CanBusManager::synchronized_final_approach(Time now) {
  auto &e=executor_;auto &s=e.status;const auto &p=*e.program;
  if(e.staged || e.sync_waiting_confirmation) {schedule_synchronized_program(now);return;}
  if(e.final_started==Time{}) {e.final_started=now;e.final_next=now+kFinalApproachDelay;e.final_command=p.target;}
  bool outside=false;std::string axes;
  for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) {
    const auto &d=devices_[config_.axes[j].address];
    if(!d.position.fresh(now,kProgramFeedbackTimeout)) return;
    const double error=p.target[j]-position_to_joint(config_.axes[j],d.position.value);
    if(std::abs(error)>config_.program_position_tolerance[j]) {outside=true;axes+=" J"+std::to_string(j+1)+"="+diagnostic_number(error*180/std::acos(-1.))+"°";}
  }
  if(!outside) return;
  if(now-e.final_started>=kFinalApproachTimeout) {
    fault("SYNC: dolot końcowy nieudany po "+std::to_string(kFinalApproachTimeout.count())+" ms; uchyb (cel − pomiar):"+axes+
        ", tolerancja ±"+diagnostic_number(config_.program_position_tolerance[0]*180/std::acos(-1.))+"°");return;
  }
  // Odpowiedź osi na zapytanie zderzyłaby się z F5 o tym samym ID: korekta po jej odebraniu.
  if(now<e.final_next || std::any_of(pending_.begin(),pending_.end(),[this](const Pending &q){return q.used && axis_index(q.address)<kAxes;})) return;
  unsigned count=0;for(bool yes:s.participating) count+=yes;
  const double cost=count*(can_frame_bits(8)+can_frame_bits(3))+can_frame_bits(2);
  const double budget=config_.bitrate*.01*std::min(kPlannedUtilization,config_.utilization_limit);
  if(spent_bits_+cost>budget) return;
  staged_streaming_states_=streaming_states_;
  for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) {
    auto axis=config_.axes[j];axis.streaming=program_streaming(j);
    const auto &d=devices_[axis.address];
    const double measured=position_to_joint(axis,d.position.value),error=p.target[j]-measured;
    if(std::abs(error)>config_.program_position_tolerance[j]) e.final_command[j]=std::clamp(e.final_command[j]+error,axis.lower,axis.upper);
    const auto sample=streaming_setpoint(axis,p.target[j],0.,staged_streaming_states_[j],measured,d.microstep.value,0.,0.,
        std::chrono::duration<double>(now-d.position.stamp).count(),e.final_command[j]);
    std::copy(sample.bytes.begin(),sample.bytes.end(),stream_payloads_[j].begin());
  }
  e.staged_reference=ProgramPoint{};e.staged_reference.seconds=p.duration;e.staged_reference.positions=p.target;
  e.staged_ack_mask=e.staged_sent_mask=0;s.sync_ack_mask=0;e.batch_started=now;
  e.due=now+kSyncPrepareMargin+2ms;e.staged=e.correction=true;s.sync_state="waiting_ack";s.sync_due_monotonic_ns=trace_ns(e.due);
  spent_bits_+=cost;
  for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) {
    e.staged_sent[j]=current_time(now);
    try {transport_->send(config_.axes[j].address,stream_payloads_[j]);++stats_.tx;}
    catch(const std::exception &error) {fault(axis_diagnostic_label(j+1,config_.axes[j].address)+": błąd wysłania korekty F5 SYNC: "+error.what());return;}
    e.staged_sent_mask|=1u<<j;sync_trace("sync_correction_prepared",e,e.staged_sent[j],config_.axes[j].address,&stream_payloads_[j]);
  }
  e.final_next=now+kFinalApproachPeriod;s.state="waiting_position";s.message="Dolot końcowy SYNC: korekta"+axes;
  trace_note("sync_final_approach",axes);motion_this_cycle_=true;
}
void CanBusManager::schedule_synchronized_program(Time now) {
  auto &e=executor_;auto &s=e.status;const auto &p=*e.program;
  if(e.sync_waiting_confirmation) {
    if(now>e.due+kSyncMaxShift) fault("SYNC: brak potwierdzenia nadania w terminie "+diagnostic_number(ms(kSyncMaxShift))+" ms; cykl="+std::to_string(e.frame));
    return;
  }
  if(e.staged) {
    if(now<e.due) return;
    if(now>e.due+kSyncLateness && !sync_shift(now,now,"spóźnienie wyzwolenia")) return;
    std::string missing;
    for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) {
      if(e.staged_ack_mask&(1u<<j)) {e.missed_acks[j]=0;continue;}
      ++stats_.sync_missed_acks;missing+=" "+axis_diagnostic_label(j+1,config_.axes[j].address);
      if(++e.missed_acks[j]>kSyncMissedAckLimit) {
        fault("SYNC: brak ACK F5=05 przed terminem od "+axis_diagnostic_label(j+1,config_.axes[j].address)+" w "+
            std::to_string(e.missed_acks[j])+" kolejnych cyklach (dopuszczalne "+std::to_string(kSyncMissedAckLimit)+"); cykl="+std::to_string(e.frame));return;
      }
    }
    if(!missing.empty()) {sync_trace("sync_ack_missing",e,now);trace_note("sync_ack_missing",missing);}
    e.late_ack_mask=e.staged_sent_mask&~e.staged_ack_mask;
    e.sync_sent=now;e.sync_waiting_confirmation=true;
    s.sync_state="waiting_tx_confirmation";s.sync_sent_monotonic_ns=trace_ns(now);
    sync_trace("sync_submitted",e,now,0,&sync_payload_);
    try {transport_->send(0,sync_payload_);++stats_.tx;}
    catch(const std::exception &error) {fault(std::string("SYNC: nie wysłano komendy wyzwolenia: ")+error.what());}
    return;
  }
  if(now+kSyncPrepareMargin>e.due && e.frame>0 && !sync_shift(now+kSyncPrepareMargin,now,"spóźnione przygotowanie zestawu")) return;
  if(now>=e.due) {fault("SYNC: pominięto termin przygotowania zestawu; cykl="+std::to_string(e.frame));return;}
  unsigned count=0;for(bool yes:s.participating) count+=yes;
  const double cost=count*(can_frame_bits(8)+can_frame_bits(3))+can_frame_bits(2);
  const double budget=config_.bitrate*.01*std::min(kPlannedUtilization,config_.utilization_limit);
  if(spent_bits_+cost>budget) {fault("SYNC: brak budżetu całego zestawu przed wysyłaniem");return;}
  const double phase=std::min(p.duration,e.frame*kStreamingCycleSeconds);
  e.staged_reference=program_reference(p,phase);
  // Ostatni punkt samplera ma seconds=duration, a nie czas granicy zaokrąglenia.
  e.staged_reference.seconds=phase;
  double horizon=std::max(0.,p.duration-phase);
  for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) horizon=std::min(horizon,config_.program_streaming[j].lookahead_time_sec);
  e.staged_horizon=program_lookahead_time(p,phase,horizon);
  const auto target=program_reference(p,e.staged_horizon);
  staged_streaming_states_=streaming_states_;e.staged_ack_mask=e.staged_sent_mask=0;
  s.sync_ack_mask=0;s.sync_state="waiting_ack";s.sync_due_monotonic_ns=trace_ns(e.due);
  e.batch_started=now;e.staged=true;
  for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) {
    auto axis=config_.axes[j];axis.streaming=program_streaming(j);
    const auto feedback=streaming_feedback_position(j,now);
    std::optional<double> equivalent;
    if(feedback.valid) {
      const double measured_phase=std::chrono::duration<double>(feedback.stamp-e.phase_origin).count();
      const double at_measurement=program_reference(p,std::clamp(measured_phase,0.,p.duration)).positions[j];
      // Podstawienie do istniejącego regulatora zachowuje podpisany uchyb
      // q_ref(t_RX)-q_meas, bez ekstrapolowania pomiaru do przyszłego SYNC.
      equivalent=e.staged_reference.positions[j]-(at_measurement-feedback.value);
      TraceEvent record;record.event="sync_feedback";record.axis=axis.address;
      record.request_ns=trace_ns(feedback.stamp);record.deadline_ns=trace_ns(e.due);
      record.position_age_ns=trace_ns(now)-trace_ns(feedback.stamp);
      std::snprintf(record.detail.data(),record.detail.size(),"cycle=%zu q_RX=%.12g q_measured=%.12g error=%.12g",e.frame,at_measurement,feedback.value,at_measurement-feedback.value);trace(record);
    }
    const auto sample=streaming_setpoint(axis,e.staged_reference.positions[j],e.staged_reference.velocities[j],
        staged_streaming_states_[j],equivalent,devices_[axis.address].microstep.value,
        e.staged_reference.accelerations[j],std::max(0.,p.duration-phase),0.,target.positions[j]);
    std::copy(sample.bytes.begin(),sample.bytes.end(),stream_payloads_[j].begin());
  }
  spent_bits_+=cost;
  for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) {
    const auto address=config_.axes[j].address;
    // Oś jeszcze odpowiada na zapytanie: F5 z tym samym ID zderzyłby się z odpowiedzią. Pomijamy ją w tym zestawie
    // (liczy się jak brak ACK; napęd jedzie dalej do poprzedniego celu).
    if(std::any_of(pending_.begin(),pending_.end(),[address](const Pending &p){return p.used && p.address==address;})) {
      sync_trace("sync_axis_skipped",e,now,address);continue;
    }
    e.staged_sent[j]=current_time(now);
    if(e.staged_sent[j]>=e.due) {fault("SYNC: przekroczony termin podczas przygotowania osi");return;}
    try {transport_->send(config_.axes[j].address,stream_payloads_[j]);++stats_.tx;}
    catch(const std::exception &error) {
      fault(axis_diagnostic_label(j+1,config_.axes[j].address)+": błąd wysłania F5 do bufora SYNC: "+error.what());return;
    }
    e.staged_sent_mask|=1u<<j;
    sync_trace("sync_prepared",e,e.staged_sent[j],config_.axes[j].address,&stream_payloads_[j]);
  }
  if(e.frame==0 && current_time(now)>e.due-kStreamingCycle) {
    fault("SYNC: pierwszy zestaw nie został przygotowany co najmniej 10 ms przed startem");return;
  }
  // Fazowanie diagnostyki odnosi się do najwcześniejszej wysyłki F5.
  for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) next_position_[config_.axes[j].address]={};
  motion_this_cycle_=true;
}
}
