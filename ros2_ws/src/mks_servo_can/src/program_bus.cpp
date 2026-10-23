#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mks_servo_can {
using namespace std::chrono_literals;
void CanBusManager::configure_program_authorization(const BusConfig &a) {
  if(active_ || manual_->busy()) throw std::logic_error("Odbiór programu wymaga postoju");
  config_.program_enabled=a.program_enabled;config_.program_revision=a.program_revision;
  config_.program_stop_verified=a.program_stop_verified;config_.program_sync_verified=a.program_sync_verified;
  config_.program_staged_reset_verified=a.program_staged_reset_verified;
  for(unsigned j=0;j<6;++j) {
    if(!config_.axes[j].address) config_.axes[j].address=j+1;
    if(std::isfinite(a.axes[j].tracking_error) && a.axes[j].tracking_error>0) config_.axes[j].tracking_error=a.axes[j].tracking_error;
    if(a.axes[j].acceleration_code) config_.axes[j].acceleration_code=a.axes[j].acceleration_code;
  }
  rebuild_axis_lookup();
}
void CanBusManager::update_motion_model(const std::string &model) {
  if(model!=motion_model_) {
    if(executor_.status.busy) fault("Model robota zmienił się podczas wykonania");
    else if(executor_.status.state=="ready") { executor_.status.state="invalidated";executor_.status.message="Model zmienił się; przelicz program"; }
    motion_model_=model;
  }
}
void CanBusManager::update_motion_settings(const std::string &revision,bool allow_missing,Time now) {
  if(revision.empty() || revision.size()>64) return;
  if(revision!=motion_settings_revision_ && executor_.status.state=="ready") {
    executor_.status.state="invalidated";executor_.status.message="Ustawienia zmieniły się; przygotuj program ponownie";
  }
  motion_settings_revision_=revision;allow_missing_motion_=allow_missing;motion_settings_stamp_=now;
}
MotionProgramStatus CanBusManager::prepare_program(PreparedMotionProgram p,Time now) {
  if(!config_.program_enabled || config_.program_revision!=profile_.revision || !config_.program_stop_verified ||
      !config_.program_sync_verified || !config_.program_staged_reset_verified)
    throw std::runtime_error("Brak odbioru wykonania dla aktywnej rewizji: STOP, synchronizacja i kasowanie przygotowanych rozkazów");
  if(p.revision!=profile_.revision || p.model_id.empty() || p.model_id!=motion_model_ || p.settings_revision!=motion_settings_revision_ ||
      now<motion_settings_stamp_ || now-motion_settings_stamp_>=1500ms)
    throw std::runtime_error("Nieaktualna konfiguracja lub ustawienia wykonania");
  // JTC musi być wcześniej zdezaktywowany; przejęcie jest możliwe wyłącznie
  // w potwierdzonym postoju i po opróżnieniu ostatniej komendy kontrolera.
  if(active_ && source_!=Source::program) {
    if(inflight_ || latest_ || !feedback_good(now)) throw std::runtime_error("Kontroler nie zwolnił komend; zakończ ruch JTC");
    for(const auto &a:config_.axes) {
      const auto &d=devices_[a.address];
      if(!d.speed.fresh(now,150ms) || d.speed.value!=0) throw std::runtime_error("Niepotwierdzony postój JTC");
    }
    active_=false;program_owned_=true;source_=Source::none;
  }
  if(!service_allowed() || scanning_ || fault_ || executor_.status.busy || !transport_->is_open())
    throw std::runtime_error("Przygotowanie wymaga postoju, zakończonego skanu i wolnego CAN");
  std::array<std::uint16_t,6> microsteps{};
  for(unsigned j=0;j<6;++j) {
    const auto &a=config_.axes[j];
    if(a.address!=j+1) throw std::runtime_error("Program wymaga mapowania CAN 1–6 na J1–J6");
    const auto &d=devices_[a.address];const bool present=d.detected && d.consecutive_timeouts<3;
    p.participating[j]=present;
    if(!present) {
      if(!allow_missing_motion_) throw std::runtime_error("Brak J"+std::to_string(j+1)+"; tryb niepełnego zestawu jest wyłączony");
      continue;
    }
    if(!profile_.axes[j].configured || !manual_->calibration_valid(j+1) || !manual_->microstep(j+1) ||
        !d.position.fresh(now,config_.feedback_timeout) || now<d.position_sent || now-d.position_sent>=config_.feedback_timeout || !d.enable.fresh(now,300ms) || !d.enable.value ||
        !d.stall.fresh(now,300ms) || d.stall.value || !d.mode.valid || !supports_motor_state(d.mode.value))
      throw std::runtime_error("J"+std::to_string(j+1)+": wymagane przygotowanie ENABLE, kalibracja i świeży stan");
    if(p.points.empty() || std::abs(p.points.front().positions[j]-position_to_joint(a,d.position.value))>a.tracking_error/4)
      throw std::runtime_error("Zmieniła się pozycja początkowa; przelicz PTP");
    microsteps[j]=manual_->microstep(j+1);
  }
  p=compile_motion_program(std::move(p),config_.axes,microsteps,std::chrono::duration<double>(config_.cycle).count());
  p.id += "-"+std::to_string(Clock::now().time_since_epoch().count())+"-"+std::to_string(++program_generation_);
  program_owned_=true;latest_.reset();
  executor_={};executor_.program=std::move(p);const auto &program=*executor_.program;
  auto &s=executor_.status;s.program_id=program.id;s.state="ready";s.mode=program.mode;s.participating=program.participating;
  s.tcp_verified=program.tcp_verified;s.duration=program.duration;s.start=program.start;s.target=program.target;
  s.message=program.tcp_verified ? "Program gotowy; oczekiwanie na zatwierdzenie ruchu" : "Program gotowy dla części osi; pozycja TCP niepotwierdzona";
  return s;
}
bool CanBusManager::program_feedback_good(Time now) const {
  for(unsigned j=0;j<6;++j) if(executor_.status.participating[j]) {
    const auto &d=devices_[config_.axes[j].address];
    if(!d.position.fresh(now,config_.feedback_timeout) || now<d.position_sent || now-d.position_sent>=config_.feedback_timeout || !d.enable.fresh(now,300ms) || !d.enable.value ||
        !d.stall.fresh(now,300ms) || d.stall.value || d.consecutive_timeouts>=3) return false;
  }
  return true;
}
bool CanBusManager::execute_program(const std::string &id,const std::string &key,Time now) {
  auto &e=executor_;auto &s=e.status;
  if(s.program_id==id && s.execution_key==key && !key.empty()) return s.state!="failed";
  if(s.state!="ready" || s.busy || !e.program || id!=s.program_id || key.empty() || key.size()>64 ||
      std::find(execution_keys_.begin(),execution_keys_.end(),key)!=execution_keys_.end() ||
      active_ || manual_->busy() || fault_ || configuration_locked_ || scanning_ || !program_feedback_good(now)) return false;
  if(e.program->revision!=profile_.revision || e.program->settings_revision!=motion_settings_revision_ ||
      now<motion_settings_stamp_ || now-motion_settings_stamp_>=1500ms || !config_.program_staged_reset_verified) return false;
  for(unsigned j=0;j<6;++j) {
    const auto &d=devices_[config_.axes[j].address];
    if(s.participating[j]) {
      if(!manual_->calibration_valid(j+1) || !profile_.axes[j].configured || !manual_->microstep(j+1) || std::abs(position_to_joint(config_.axes[j],d.position.value)-e.program->start[j])>config_.axes[j].tracking_error/4) return false;
    } else if(d.detected && d.consecutive_timeouts<3) return false; // Zmienił się zestaw osi.
  }
  execution_keys_[execution_key_cursor_++ % execution_keys_.size()]=key;
  s.execution_key=key;s.busy=true;s.state="arming";s.message="Weryfikacja postoju i przygotowanie synchronizacji";
  e.preparation_deadline=now+3s;e.setup_axis=0;e.setup_step=0;e.waiting=false;
  active_=true;source_=Source::program;stop_sent_=false;latest_.reset();inflight_.reset();
  staged_.fill(false);sent_stage_.fill(false);
  for(auto &pending:pending_) pending.used=false;
  manual_->invalidate_program_handover();
  return true;
}
void CanBusManager::program_setup_reply(const Frame &frame,Time) {
  auto &e=executor_;
  if(e.status.state!="arming" || !e.waiting || frame.address!=config_.axes[e.setup_axis].address) {
    fault("Nieoczekiwane potwierdzenie uzbrojenia");return;
  }
  bool good=false;
  switch(e.setup_step) {
    case 0: { auto speed=parse_speed(frame.address,frame.data);good=speed && *speed==0;break; }
    case 1: { auto state=parse_motor_state(frame.address,frame.data);good=state && *state==1;break; }
    case 2: good=frame.data[1]==1;break;
    case 3: good=frame.data[1]==1;break;
    case 5: good=frame.data[1]==manual_->microstep(e.setup_axis+1);break;
    case 6: good=supports_motor_state(frame.data[1]);break;
    case 4: {
      std::uint32_t timeout=0;for(unsigned i=1;i<5;++i) timeout=(timeout<<8)|frame.data[i];
      good=timeout>=3*static_cast<unsigned>(config_.cycle.count()) && timeout<=1000;break;
    }
  }
  if(!good) { fault("Niepotwierdzony postój, synchronizacja lub watchdog napędu");return; }
  e.waiting=false;++e.setup_step;
  if(e.setup_step==7) { ++e.setup_axis;e.setup_step=0; }
}
void CanBusManager::schedule_program(Time now) {
  auto &e=executor_;auto &s=e.status;
  if(!s.busy || !e.program || !program_feedback_good(now)) { fault("Utrata sprzężenia zwrotnego uczestniczącego napędu");return; }
  if(s.state=="arming") {
    if(now>=e.preparation_deadline) { fault("Przekroczony termin uzbrojenia programu");return; }
    if(e.waiting) return;
    while(e.setup_axis<6 && !s.participating[e.setup_axis]) ++e.setup_axis;
    if(e.setup_axis==6) {
      s.state="warming";e.warmup_started=now;
      for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) next_position_[config_.axes[j].address]={};
      return;
    }
    const auto address=config_.axes[e.setup_axis].address;
    if(!can_send(address,now)) return;
    std::vector<std::uint8_t> data;std::size_t length=3;
    switch(e.setup_step) {
      case 0: data=request(address,0x32);length=4;break;
      case 1: data=request(address,0xf1);break;
      case 2: data=request(address,0x4a,{1});break;
      case 3: data=parameter_request(address,0x4a);break;
      case 4: data=parameter_request(address,0x98);length=6;break;
      case 5: data=parameter_request(address,0x84);break;
      default: data=parameter_request(address,0x82);break;
    }
    if(!reserve(data.size(),length,true)) return;
    transport_->send(address,data);++stats_.tx;
    for(auto &pending:pending_) if(!pending.used) {
      pending={true,address,Kind::program_setup,static_cast<std::uint8_t>(e.setup_step==3 ? 0x4a : e.setup_step==4 ? 0x98 : e.setup_step==5 ? 0x84 : e.setup_step==6 ? 0x82 : data.front()),length,now,now+100ms};
      e.waiting=true;break;
    }
    return;
  }
  if(s.state=="warming") {
    if(now>=e.preparation_deadline) { fault("Przekroczony termin odświeżenia pozycji przed PTP");return; }
    for(unsigned j=0;j<kAxes;++j) if(s.participating[j] && devices_[config_.axes[j].address].position_sent<e.warmup_started) return;
    if(std::any_of(pending_.begin(),pending_.end(),[](const auto &p){return p.used;})) return;
    s.state="staging";e.frame=0;e.phase_origin=next_cycle_;e.started=e.due=e.phase_origin+config_.cycle;
    staged_.fill(false);sent_stage_.fill(false);
    // Rozłożenie okresowych odczytów; następne terminy przesuwamy o okres, nie o opóźnienie pętli.
    unsigned rank=0,count=0;for(bool participating:s.participating) if(participating) ++count;
    for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) {
      const auto address=config_.axes[j].address;
      next_position_[address]=e.phase_origin+kStageWindow+(rank%2 ? config_.cycle : 0ms);
      next_enable_[address]=e.phase_origin+kFeedbackWindowEnd+std::chrono::milliseconds(rank*100/count);
      next_stall_[address]=e.phase_origin+kFeedbackWindowEnd+std::chrono::milliseconds((rank*100/count+50)%100);
      next_speed_[address]=e.phase_origin+kFeedbackWindowEnd+std::chrono::milliseconds(rank*50/count);
      next_motor_[address]=e.phase_origin+kFeedbackWindowEnd+std::chrono::milliseconds((rank*50/count+25)%50);
      ++rank;
    }
    return;
  }
  const auto &program=*e.program;
  s.elapsed=std::max(0.,std::chrono::duration<double>(now-e.started).count());
  s.progress=std::min(1.,s.elapsed/program.duration);
  for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) {
    const auto &a=config_.axes[j];const auto &d=devices_[a.address];
    if(d.position.stamp<=e.checked_position[j]) continue;
    e.checked_position[j]=d.position.stamp;
    const auto bounds=program_position_bounds(program,j,std::chrono::duration<double>(d.position_sent-e.started).count(),
        std::chrono::duration<double>(d.position.stamp-e.started).count());
    const auto actual=position_to_joint(a,d.position.value);
    if(actual<bounds[0]-a.tracking_error || actual>bounds[1]+a.tracking_error) {
      fault("Przekroczony błąd śledzenia J"+std::to_string(j+1));return;
    }
  }
  if(e.frame<program.frames.size()) {
    now=current_time(now);
    auto all_staged=[&] { for(unsigned j=0;j<kAxes;++j) if(s.participating[j] && !staged_[j]) return false;return true; };
    if(now>e.due+1ms) { ++stats_.missed_cycles;fault("Przekroczony termin SYNC; program przerwany bez nadrabiania próbek");return; }
    if(now>=e.due) {
      if(!all_staged()) { fault("Brak kompletu ACK przed terminem SYNC");return; }
      if(!reserve(2,0,true)) { fault("Brak budżetu CAN na terminowy SYNC");return; }
      now=current_time(now);
      if(now>e.due+1ms) { ++stats_.missed_cycles;fault("Przekroczony termin przed zapisem SYNC");return; }
      transport_->send(0,sync_payload_);++stats_.tx;++stats_.committed;
      stats_.sync_lateness_ms.add(std::chrono::duration<double,std::milli>(now-e.due).count());
      ++e.frame;staged_.fill(false);sent_stage_.fill(false);s.state="running";s.message="Wykonywanie zatwierdzonego PTP";
      if(e.frame<program.frames.size()) e.due=e.started+config_.cycle*e.frame;
    }
    if(e.frame<program.frames.size() && now>=e.phase_origin) {
      const auto stage_end=cycle_start_+kStageWindow;
      if(now>=stage_end) {
        if(!all_staged()) { fault("Niepełny zestaw ACK w oknie 0–8 ms");return; }
      } else {
        const auto &frame=program.frames[e.frame];
        for(unsigned j=0;j<kAxes;++j) if(s.participating[j] && !sent_stage_[j]) {
          const auto address=config_.axes[j].address;now=current_time(now);
          if(now>=stage_end) { fault("Przekroczone okno wysyłania F5");return; }
          if(!can_send(address,now)) continue;
          if(!reserve(frame.payloads[j].size(),3,true,1)) { fault("Brak budżetu F5 w oknie przygotowania");return; }
          transport_->send(address,frame.payloads[j]);++stats_.tx;sent_stage_[j]=true;
          for(auto &pending:pending_) if(!pending.used) {
            pending={true,address,Kind::stage,0xf5,3,now,std::min(now+config_.response_timeout,stage_end)};break;
          }
        }
      }
    }
  }
  if(s.elapsed>=program.duration && e.frame==program.frames.size()) {
    s.state="settling";bool reached=true;Time newest{};
    for(unsigned j=0;j<6;++j) if(s.participating[j]) {
      const auto &a=config_.axes[j];const auto &d=devices_[a.address];newest=std::max(newest,d.position.stamp);
      if(d.position_sent<e.started+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(program.duration)) ||
          std::abs(position_to_joint(a,d.position.value)-program.target[j])>a.tracking_error/4 ||
          !d.speed.fresh(now,150ms) || d.speed.value!=0) reached=false;
    }
    if(reached && newest>e.last_settle_sample) {
      if(e.settled==Time{}) e.settled=now;
      e.last_settle_sample=newest;
      if(now-e.settled>=100ms) {
        s.busy=false;s.state="succeeded";s.progress=1;s.message=s.tcp_verified ? "PTP zakończony; cel osi potwierdzony" : "PTP zakończony dla uczestniczących osi; TCP niepotwierdzone";
        active_=false;source_=Source::none;
      }
    } else if(!reached) e.settled={};
    if(s.busy && s.elapsed>program.duration+1.) fault("Niepotwierdzone zakończenie PTP w terminie");
  }
}
}
