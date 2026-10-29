#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/diagnostic_message.hpp"
#include "mks_servo_can/trace.hpp"
#include <algorithm>
#include <cstdio>

namespace mks_servo_can {
using namespace std::chrono_literals;
namespace {
constexpr auto kSyncLateness=2500us;
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
  if(!e.staged || e.sync_waiting_confirmation || stamp<e.staged_sent[j] || stamp>=e.due ||
      frame.data[1]!=5 || (e.staged_ack_mask&(1u<<j))) {
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
  if(!e.sync_waiting_confirmation || stamp<e.sync_sent || stamp>e.due+kSyncLateness) {
    fault("SYNC: niezgodne lub spóźnione potwierdzenie nadania; cykl="+std::to_string(e.frame));return;
  }
  sync_trace("sync_activated",e,stamp);
  for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) sync_trace("sync_reference_committed",e,stamp,config_.axes[j].address,&stream_payloads_[j]);
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
void CanBusManager::schedule_synchronized_program(Time now) {
  auto &e=executor_;auto &s=e.status;const auto &p=*e.program;
  if(e.sync_waiting_confirmation) {
    if(now>e.due+kSyncLateness) fault("SYNC: brak potwierdzenia nadania w terminie 2,5 ms; cykl="+std::to_string(e.frame));
    return;
  }
  if(e.staged) {
    if(now<e.due) return;
    if(now>e.due+kSyncLateness) {
      fault("SYNC: spóźnienie wyzwolenia="+diagnostic_number(std::chrono::duration<double,std::milli>(now-e.due).count())+
          " ms, limit 2,5 ms; cykl="+std::to_string(e.frame));return;
    }
    if(e.staged_ack_mask!=participants(s)) {
      std::string missing;
      for(unsigned j=0;j<kAxes;++j) if(s.participating[j] && !(e.staged_ack_mask&(1u<<j)))
        missing+=" "+axis_diagnostic_label(j+1,config_.axes[j].address);
      fault("SYNC: brak ACK F5=05 przed terminem od"+missing+"; cykl="+std::to_string(e.frame));return;
    }
    e.sync_sent=now;e.sync_waiting_confirmation=true;
    s.sync_state="waiting_tx_confirmation";s.sync_sent_monotonic_ns=trace_ns(now);
    sync_trace("sync_submitted",e,now,0,&sync_payload_);
    try {transport_->send(0,sync_payload_);++stats_.tx;}
    catch(const std::exception &error) {fault(std::string("SYNC: nie wysłano komendy wyzwolenia: ")+error.what());}
    return;
  }
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
  staged_streaming_states_=streaming_states_;e.staged_ack_mask=0;
  s.sync_ack_mask=0;s.sync_state="waiting_ack";s.sync_due_monotonic_ns=trace_ns(e.due);
  e.batch_started=now;e.staged=true;
  for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) {
    auto axis=config_.axes[j];axis.streaming=config_.program_streaming[j];
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
    e.staged_sent[j]=current_time(now);
    if(e.staged_sent[j]>=e.due) {fault("SYNC: przekroczony termin podczas przygotowania osi");return;}
    try {transport_->send(config_.axes[j].address,stream_payloads_[j]);++stats_.tx;}
    catch(const std::exception &error) {
      fault(axis_diagnostic_label(j+1,config_.axes[j].address)+": błąd wysłania F5 do bufora SYNC: "+error.what());return;
    }
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
