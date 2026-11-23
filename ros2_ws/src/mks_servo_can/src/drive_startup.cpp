// Inicjalizacja napędów po starcie systemu i po powrocie napędu po utracie zasilania.
// Napęd MKS pamięta ostatni stan ENABLE, więc pierwszą komendą jest F3=0, wysyłana także do osi jeszcze niewykrytych.
// Kolejność na oś: F3=0 → potwierdzenie odczytem 3A → F7 (kasowanie poprzedniego ruchu) → 8C=00/00, gdy trzeba →
// potwierdzenie 8C, 4A=0, 84 i 82 odczytami tła. SYNC (4A) jest tylko sprawdzany: pozostawione SYNC=1 ma trafić do
// istniejącej ścieżki awarii, a nie zostać ukryte zapisem. Do końca inicjalizacji oś nie może dostać ENABLE=1.
#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/diagnostic_message.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include "mks_servo_can/trace.hpp"

using namespace std::chrono_literals;
namespace mks_servo_can {
namespace {
constexpr auto kDisableRetry=300ms;       // napęd odpowiada, ale nie potwierdził jeszcze ENABLE=0
constexpr auto kSilentDisableRetry=1s;    // napęd milczy: F3=0 trafi do niego zaraz po włączeniu zasilania
constexpr auto kIdentityProbePeriod=1s;   // napęd włączony później niż system nie zostałby wykryty przez jednorazowy skan
constexpr auto kResponseModeRetry=300ms;
constexpr auto kInitializationTimeout=5s;
constexpr auto kDiscoveryGrace=10s;       // czas na uzgodnienie konfiguracji i skan: brak napędu jeszcze nie jest wynikiem
constexpr auto kDriveRestartSilence=2s;   // krótsza cisza to zakłócenie, nie ponowne włączenie napędu
constexpr std::uint8_t kExpectEnable=1,kExpectStop=2,kExpectResponseMode=4;
}

void CanBusManager::arm_drive_startup(unsigned joint,const char *reason) {
  auto &state=startup_[joint];state={};state.pending=true;
  startup_ready_&=~(1u<<joint);next_enable_[joint+1]={};
  manual_->forget_preparation(joint+1);
  trace_note("drive_startup_armed",reason,joint+1,0xf3);
}

void CanBusManager::arm_drive_startup_all(Time now) {
  if(!config_.drive_startup || startup_armed_) return;
  // Tylko pierwsze otwarcie w procesie: reset CAN i ponowne połączenie nie są włączeniem systemu.
  startup_armed_=true;startup_grace_=now+kDiscoveryGrace;
  for(unsigned j=0;j<kAxes;++j) arm_drive_startup(j,"start systemu");
}

void CanBusManager::note_drive_contact(std::uint16_t address,Time stamp) {
  if(!config_.drive_startup || address<1 || address>kAxes) return;
  const auto &device=devices_[address];
  // Wymagane nieudane zapytania: oś nieodpytywana podczas programu też ma stary last_ok, ale nie zniknęła.
  if(device.consecutive_timeouts<3 || device.last_ok==Time{} || stamp<device.last_ok+kDriveRestartSilence) return;
  arm_drive_startup(address-1,"powrót napędu po ciszy");
}

bool CanBusManager::send_startup_frame(std::uint16_t address,const std::vector<std::uint8_t> &data,Time now) {
  if(!can_send(address,now) || !reserve(data.size(),3,true)) return false;
  transport_->send(address,data);++stats_.tx;
  return true;
}

void CanBusManager::schedule_drive_startup(Time now) {
  if(!config_.drive_startup || !transport_->is_open() || active_ || executor_.status.busy || (!fault_ && manual_->busy())) return;
  for(unsigned j=0;j<kAxes;++j) {
    auto &state=startup_[j];if(!state.pending) continue;
    const auto address=static_cast<std::uint16_t>(j+1);auto &device=devices_[address];
    const bool responding=device.detected && device.consecutive_timeouts<3;
    if(responding && state.deadline==Time{}) state.deadline=now+kInitializationTimeout;
    const bool disabled=state.disable_sent!=Time{} && device.enable.valid && !device.enable.value && device.enable.stamp>state.disable_sent;
    if(!disabled) {
      if(now>=state.next_disable && send_startup_frame(address,enable_request(address,false),now)) {
        state.disable_sent=now;state.expected|=kExpectEnable;
        state.next_disable=now+(responding ? kDisableRetry : kSilentDisableRetry);next_enable_[address]={};
      }
      if(!device.detected && !fault_ && !scanning_ && configuration_ready_ && !configuration_locked_ &&
          now>=state.next_probe && send_query(address,Kind::identity,now,false)) state.next_probe=now+kIdentityProbePeriod;
    } else if(!fault_) {
      if(!state.stop_sent) {
        if(send_startup_frame(address,stop_request(address),now)) {state.stop_sent=true;state.expected|=kExpectStop;}
      } else if(device.responses_known && !device.responses_disabled) {
        if(now>=state.next_response_mode && send_startup_frame(address,request(address,0x8c,{0,0}),now)) {
          state.expected|=kExpectResponseMode;state.next_response_mode=now+kResponseModeRetry;
        }
      } else if(device.responses_known && device.synchronization_known && device.synchronization_disabled &&
          device.microstep.valid && device.mode.valid) {
        state.pending=state.failed=false;startup_ready_|=1u<<j;++startup_generation_;
        trace_note("drive_startup_ready",axis_diagnostic_label(j+1,address),address,0xf3);
        continue;
      }
    }
    if(responding && !state.failed && now>=state.deadline) {
      // Bez awarii magistrali: oś pozostaje zablokowana dla ENABLE=1, a F3=0 jest dalej ponawiane.
      state.failed=true;trace_note("drive_startup_failed",axis_diagnostic_label(j+1,address)+": brak potwierdzenia inicjalizacji",address,0xf3);
    }
  }
}

bool CanBusManager::drive_startup_reply(const Frame &frame) {
  if(!config_.drive_startup || frame.address<1 || frame.address>kAxes || frame.data.size()!=3) return false;
  const auto code=frame.data[0];
  const std::uint8_t bit=code==0xf3 ? kExpectEnable : code==0xf7 ? kExpectStop : code==0x8c ? kExpectResponseMode : 0;
  auto &state=startup_[frame.address-1];
  if(!bit || !(state.expected&bit) || !validate_response(frame.address,frame.data,code,3).valid) return false;
  // F3 bywa ponawiane, więc jego potwierdzenia są oczekiwane do końca inicjalizacji osi.
  if(bit!=kExpectEnable || !state.pending) state.expected&=static_cast<std::uint8_t>(~bit);
  if(bit==kExpectResponseMode && frame.data[1]==1) devices_[frame.address].responses_known=false;  // odczyt kontrolny 8C
  return true;
}

bool CanBusManager::drive_startup_pending(unsigned axis) const {
  return config_.drive_startup && axis>=1 && axis<=kAxes && startup_[axis-1].pending;
}

void CanBusManager::fill_startup_snapshot(BusSnapshot &snapshot) const {
  snapshot.startup_enabled=config_.drive_startup;snapshot.startup_generation=startup_generation_;
  for(unsigned j=0;j<kAxes;++j) {
    const auto bit=1u<<j;const auto &state=startup_[j];const auto &device=devices_[j+1];
    const bool responding=device.detected && device.consecutive_timeouts<3;
    if(state.pending) {
      snapshot.startup_pending|=bit;
      if(state.failed) snapshot.startup_failed|=bit;
      else if(responding || last_tick_<startup_grace_) snapshot.startup_busy|=bit;
    } else if((startup_ready_&bit) && responding) snapshot.startup_ready|=bit;
  }
}
}  // namespace mks_servo_can
