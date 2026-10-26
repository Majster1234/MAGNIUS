#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mks_servo_can {
using namespace std::chrono_literals;
void CanBusManager::configure_program_authorization(const BusConfig &a) {
  if(active_ || manual_->busy()) throw std::logic_error("Konfiguracja programu wymaga postoju");
  if(a.use_ack || a.program_position_period!=kStreamingCycle ||
      std::any_of(a.program_position_tolerance.begin(),a.program_position_tolerance.end(),
        [](double value){return !std::isfinite(value) || value<=0;}))
    throw std::invalid_argument("Streaming wymaga use_ack=false, pozycji 100 Hz i dodatnich tolerancji końca");
  config_.program_enabled=a.program_enabled;
  config_.program_position_period=kStreamingCycle;
  config_.program_position_tolerance=a.program_position_tolerance;
  config_.program_streaming=a.program_streaming;
  // Pola ACK/odbioru są przyjmowane przez stare API, ale nie sterują ruchem.
  for(unsigned j=0;j<kAxes;++j) {
    if(!config_.axes[j].address) config_.axes[j].address=j+1;
    if(a.axes[j].acceleration_code) config_.program_streaming[j].acceleration=a.axes[j].acceleration_code;
  }
  rebuild_axis_lookup();
}
void CanBusManager::update_motion_model(const std::string &model) {
  if(model==motion_model_) return;
  if(executor_.status.busy || source_==Source::tcp_jog) fault("Model robota zmienił się podczas wykonania");
  else if(executor_.status.state=="ready") { executor_.status.state="invalidated";executor_.status.message="Model zmienił się; przelicz program"; }
  motion_model_=model;
}
void CanBusManager::update_motion_settings(const std::string &revision,bool allow_missing,Time now) {
  if(revision.empty() || revision.size()>64) return;
  if(source_==Source::tcp_jog && revision!=motion_settings_revision_) fault("Ustawienia zmieniły się podczas JOG TCP");
  if(revision!=motion_settings_revision_ && executor_.status.state=="ready") {
    executor_.status.state="invalidated";executor_.status.message="Ustawienia zmieniły się; przygotuj program ponownie";
  }
  motion_settings_revision_=revision;allow_missing_motion_=allow_missing;motion_settings_stamp_=now;
}
MotionProgramStatus CanBusManager::prepare_program(PreparedMotionProgram p,Time now) {
  if(source_==Source::tcp_jog) throw std::runtime_error("Zakończ JOG TCP przed przygotowaniem programu");
  if(!config_.program_enabled) throw std::runtime_error("Wykonanie PTP/LIN wyłączone");
  // Rewizję wybiera zakończona transakcja konfiguracji, a nie kopia UUID w YAML.
  // Sprawdzamy tę bramkę przed przejęciem sterowania od JTC.
  if(!configuration_ready_ || configuration_locked_)
    throw std::runtime_error("Konfiguracja nie jest uzgodniona; zakończ zapis lub uzgadnianie konfiguracji");
  if(p.revision!=profile_.revision || p.model_id.empty() || p.model_id!=motion_model_ || p.settings_revision!=motion_settings_revision_ ||
      now<motion_settings_stamp_ || now-motion_settings_stamp_>=1500ms)
    throw std::runtime_error("Nieaktualna konfiguracja lub ustawienia wykonania");
  if(active_ && source_!=Source::program) {
    if(latest_ || !feedback_good(now)) throw std::runtime_error("Kontroler nie zwolnił komend; zakończ ruch JTC");
    for(const auto &a:config_.axes) {
      const auto &d=devices_[a.address];
      if(!d.motor_state.fresh(now,150ms) || d.motor_state.value!=1) throw std::runtime_error("Niepotwierdzony postój JTC");
    }
    active_=false;program_owned_=true;source_=Source::none;
  }
  if(!service_allowed() || scanning_ || fault_ || executor_.status.busy || !transport_->is_open())
    throw std::runtime_error("Przygotowanie wymaga postoju, zakończonego skanu i wolnego CAN");
  std::array<std::uint16_t,6> microsteps{};auto axes=config_.axes;unsigned participants=0;
  for(unsigned j=0;j<kAxes;++j) {
    const auto &a=config_.axes[j];
    if(a.address!=j+1) throw std::runtime_error("Program wymaga mapowania CAN 1–6 na J1–J6");
    const auto &d=devices_[a.address];p.participating[j]=d.detected && d.consecutive_timeouts<3;
    if(!p.participating[j]) {
      if(!allow_missing_motion_) throw std::runtime_error("Brak J"+std::to_string(j+1));
      continue;
    }
    ++participants;
    if(!profile_.axes[j].configured || !manual_->calibration_valid(j+1) || !d.microstep.valid ||
        !d.position.fresh(now,kProgramFeedbackTimeout) || !d.enable.fresh(now,300ms) || !d.enable.value ||
        !d.stall.fresh(now,300ms) || d.stall.value || !d.mode.valid || d.mode.value!=5)
      throw std::runtime_error("J"+std::to_string(j+1)+": wymagane ENABLE, kalibracja i świeży stan SR_vFOC");
    if(p.points.empty() || std::abs(p.points.front().positions[j]-position_to_joint(a,d.position.value))>config_.program_position_tolerance[j])
      throw std::runtime_error("Zmieniła się pozycja początkowa; przelicz ruch");
    microsteps[j]=d.microstep.value;axes[j].streaming=config_.program_streaming[j];
  }
  if(streaming_bus_utilization(participants,config_.bitrate)>std::min(kPlannedUtilization,config_.utilization_limit)+1e-12)
    throw std::runtime_error("Streaming 100 Hz przekracza budżet robota 40%; sześć osi wymaga profilu 1 Mbit/s");
  p=compile_streaming_program(std::move(p),axes,microsteps);
  p.id+="-"+std::to_string(++program_generation_)+"-"+std::to_string(Clock::now().time_since_epoch().count());
  program_owned_=true;latest_.reset();executor_={};executor_.program=std::move(p);const auto &program=*executor_.program;
  auto &s=executor_.status;s.program_id=program.id;s.state="ready";s.mode=program.mode;s.participating=program.participating;
  s.tcp_verified=program.tcp_verified;s.duration=program.duration;s.start=program.start;s.target=program.target;
  s.message="Streaming 100 Hz gotowy; oczekiwanie na zatwierdzenie";
  return s;
}
bool CanBusManager::program_feedback_good(Time now) const {
  for(unsigned j=0;j<kAxes;++j) if(executor_.status.participating[j]) {
    const auto &d=devices_[config_.axes[j].address];
    if(!d.position.fresh(now,kProgramFeedbackTimeout) || !d.enable.fresh(now,300ms) || !d.enable.value ||
        !d.stall.fresh(now,300ms) || d.stall.value) return false;
  }
  return true;
}
bool CanBusManager::execute_program(const std::string &id,const std::string &key,Time now) {
  auto &e=executor_;auto &s=e.status;
  if(s.program_id==id && s.execution_key==key && !key.empty()) return s.state!="failed";
  if(s.state!="ready" || s.busy || !e.program || id!=s.program_id || key.empty() || key.size()>64 ||
      std::find(execution_keys_.begin(),execution_keys_.end(),key)!=execution_keys_.end() || active_ || manual_->busy() ||
      fault_ || configuration_locked_ || scanning_ || !program_feedback_good(now)) return false;
  if(e.program->revision!=profile_.revision || e.program->settings_revision!=motion_settings_revision_ ||
      now<motion_settings_stamp_ || now-motion_settings_stamp_>=1500ms) return false;
  for(unsigned j=0;j<kAxes;++j) {
    const auto &d=devices_[config_.axes[j].address];
    if(s.participating[j]) {
      if(!manual_->calibration_valid(j+1) || std::abs(position_to_joint(config_.axes[j],d.position.value)-e.program->start[j])>config_.program_position_tolerance[j]) return false;
    } else if(d.detected && d.consecutive_timeouts<3) return false;
  }
  execution_keys_[execution_key_cursor_++%execution_keys_.size()]=key;
  s.execution_key=key;s.busy=true;s.state="arming";s.message="Weryfikacja postoju i CanRSP=Disable";
  e.preparation_deadline=now+3s;e.setup_axis=e.setup_step=0;e.waiting=false;
  active_=true;source_=Source::program;stop_sent_=false;latest_.reset();
  streaming_states_={};
  for(auto &pending:pending_) pending.used=false;
  manual_->invalidate_program_handover();return true;
}
void CanBusManager::program_setup_reply(const Frame &frame,Time) {
  auto &e=executor_;
  if(e.status.state!="arming" || !e.waiting || frame.address!=config_.axes[e.setup_axis].address) { fault("Nieoczekiwane potwierdzenie uzbrojenia");return; }
  bool good=false;
  switch(e.setup_step) {
    case 0:good=bool(parse_speed(frame.address,frame.data));break;
    case 1:{const auto state=parse_motor_state(frame.address,frame.data);good=state && *state==1;break;}
    case 2:case 4:good=frame.data[1]==1;break;
    case 3:good=frame.data[1]==0 && frame.data[2]==0;devices_[frame.address].responses_disabled=good;break;
    case 5:good=frame.data[1]==0;devices_[frame.address].synchronization_disabled=good;break;
    case 6:good=frame.data[1]==0 && frame.data[2]==0 && frame.data[3]==1 && frame.data[4]==44;break;
    case 7:good=frame.data[1]==devices_[frame.address].microstep.value;break;
    case 8:good=frame.data[1]==5;break;
  }
  if(!good) { fault("Niepotwierdzony postój, CanRSP=Disable, SR_vFOC lub watchdog");return; }
  e.waiting=false;if(++e.setup_step==9) {++e.setup_axis;e.setup_step=0;}
}
void CanBusManager::begin_program_check(Time now) {
  auto &e=executor_;e.checking_position=true;e.check_started=now;
  for(unsigned j=0;j<kAxes;++j) if(e.status.participating[j]) next_position_[config_.axes[j].address]={};
}
bool CanBusManager::check_program_positions(Time now) {
  auto &e=executor_;bool fresh=true,within=true;std::string missing,outside;
  for(unsigned j=0;j<kAxes;++j) if(e.status.participating[j]) {
    const auto &a=config_.axes[j];const auto &d=devices_[a.address];
    if(!d.position.fresh(now,kProgramFeedbackTimeout) || d.position_sent<e.check_started) {fresh=false;missing+=" J"+std::to_string(j+1);continue;}
    if(std::abs(position_to_joint(a,d.position.value)-e.program->target[j])>config_.program_position_tolerance[j]) {within=false;outside+=" J"+std::to_string(j+1);}
  }
  if(!fresh) {e.status.state="waiting_feedback";e.status.message="Dolot końcowy: brak pomiarów"+missing;return false;}
  if(!within) {e.settled={};e.status.state="waiting_position";e.status.message="Dolot końcowy: poza tolerancją"+outside;return false;}
  e.checking_position=false;e.next_check=now+100ms;return true;
}
void CanBusManager::schedule_program(Time now) {
  auto &e=executor_;auto &s=e.status;if(!s.busy || !e.program) return;
  for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) {
    const auto &d=devices_[config_.axes[j].address];
    if((d.enable.fresh(now,300ms) && !d.enable.value) || (d.stall.fresh(now,300ms) && d.stall.value)) {fault("Napęd zgłosił ENABLE=0 lub stall");return;}
  }
  if(s.state=="arming") {
    if(now>=e.preparation_deadline) {fault("Przekroczony termin uzbrojenia");return;}
    if(e.waiting) return;
    while(e.setup_axis<kAxes && !s.participating[e.setup_axis]) ++e.setup_axis;
    if(e.setup_axis==kAxes) {
      for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) {
        const auto &d=devices_[config_.axes[j].address];
        if(!d.position.fresh(now,kProgramFeedbackTimeout) || std::abs(position_to_joint(config_.axes[j],d.position.value)-e.program->start[j])>config_.program_position_tolerance[j]) {fault("Pozycja startowa zmieniła się podczas uzbrajania");return;}
      }
      s.state="running";s.message="Streaming 100 Hz; bez ACK i kontroli punktów pośrednich";return;
    }
    const auto address=config_.axes[e.setup_axis].address;if(!can_send(address,now)) return;
    std::vector<std::uint8_t> data;std::size_t length=3;std::uint8_t command{};
    switch(e.setup_step) {
      case 0:data=request(address,0x32);length=4;command=0x32;break;
      case 1:data=request(address,0xf1);command=0xf1;break;
      case 2:data=request(address,0x8c,{0,0});command=0x8c;break;
      case 3:data=parameter_request(address,0x8c);length=4;command=0x8c;break;
      case 4:data=request(address,0x4a,{0});command=0x4a;break;
      case 5:data=parameter_request(address,0x4a);command=0x4a;break;
      case 6:data=parameter_request(address,0x98);length=6;command=0x98;break;
      case 7:data=parameter_request(address,0x84);command=0x84;break;
      default:data=parameter_request(address,0x82);command=0x82;break;
    }
    if(!reserve(data.size(),length,true)) return;
    transport_->send(address,data);++stats_.tx;
    for(auto &pending:pending_) if(!pending.used) {pending={true,address,Kind::program_setup,command,length,now,now+100ms};e.waiting=true;break;}
    return;
  }
  const auto &program=*e.program;
  s.elapsed=e.started==Time{} ? 0 : std::max(0.,std::chrono::duration<double>(now-e.started).count());
  s.progress=std::min(.99,s.elapsed/program.duration);
  if(e.frame==program.frames.size()) {
    if(!e.checking_position && now>=e.next_check) begin_program_check(now);
    if(!e.checking_position || !check_program_positions(now)) return;
    if(e.settled==Time{}) e.settled=now;
    if(now-e.settled<100ms) {s.state="settling";return;}
    s.busy=false;s.state="succeeded";s.progress=1;s.message="Ruch zakończony: pozycje końcowe potwierdzone";
    active_=false;source_=Source::none;return;
  }
  // Czas całkowitoliczbowy: double(k*0,01) przesuwało część granic o tick 1 ms.
  const auto elapsed=e.started==Time{} ? Clock::duration::zero() : now-e.started;
  const auto index=std::min(program.frames.size()-1,static_cast<std::size_t>(elapsed/kStreamingCycle));
  if(index<e.frame) return;
  unsigned participants=0;for(bool yes:s.participating) if(yes) ++participants;
  const auto cost=participants*can_frame_bits(8);
  const auto budget=config_.bitrate*std::chrono::duration<double>(config_.cycle).count()*std::min(kPlannedUtilization,config_.utilization_limit);
  if(spent_bits_+cost>budget) {++stats_.deferred;s.state="waiting_budget";s.message="Oczekiwanie na budżet zestawu";return;}
  // Referencja dla rzeczywistej chwili wysyłania, nie najbliższego starego węzła.
  const auto point=program_reference(program,std::min(program.duration,std::chrono::duration<double>(elapsed).count()));
  auto next_streaming=streaming_states_;
  for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) {
    auto axis=config_.axes[j];axis.streaming=config_.program_streaming[j];
    const auto feedback=streaming_feedback_position(j,now);
    const auto sample=streaming_setpoint(axis,point.positions[j],point.velocities[j],next_streaming[j],
        feedback.valid ? std::optional<double>(feedback.value) : std::nullopt,devices_[axis.address].microstep.value,
        point.accelerations[j],std::max(0.,program.duration-std::chrono::duration<double>(elapsed).count()),
        feedback.valid ? std::chrono::duration<double>(now-feedback.stamp).count() : 0.);
    std::copy(sample.bytes.begin(),sample.bytes.end(),stream_payloads_[j].begin());
  }
  spent_bits_+=cost;
  if(e.started==Time{}) {
    e.started=now;
    // Pierwszy odczyt dopiero po F5, kolejne w tej samej fazie10 ms.
    for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) next_position_[config_.axes[j].address]={};
  }
  // Jedna aktualna klatka. Po opóźnieniu nie odtwarzamy kolejki starych celów.
  stats_.dropped_commands+=index-e.frame;
  for(unsigned j=0;j<kAxes;++j) if(s.participating[j]) send_stream_payload(j,stream_payloads_[j]);
  streaming_states_=next_streaming;
  e.frame=index+1;e.last_sync=now;++stats_.committed;
  s.state="running";s.message="Streaming 100 Hz: lookahead, korekcja pozycji i dithering RPM";
  if(e.frame==program.frames.size()) {s.state="settling";e.next_check=now;begin_program_check(now);}
}
}
