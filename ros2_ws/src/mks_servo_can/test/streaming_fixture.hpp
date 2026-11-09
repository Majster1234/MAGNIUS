#pragma once
#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include <algorithm>
#include <deque>
#include <map>
#include <stdexcept>
using namespace mks_servo_can;
using namespace std::chrono_literals;
namespace streaming_test {
// Ten sam adapter benchmarku działa przed wprowadzeniem generacji i po nim.
template<class B,class M> auto tag_generation(B &batch,const M &manager,int)
    -> decltype(batch.configuration_generation=manager.control_state().configuration_generation,void()) {
  batch.configuration_generation=manager.control_state().configuration_generation;
}
template<class B,class M> void tag_generation(B &,const M &,long) {}
class Port final : public Transport {
 public:
  bool opened{},disconnected{},echo{},bad_response_mode{},bad_sync_mode{},bad_watchdog{};
  unsigned absent{},extra_absent{},missing_position{},stalled{};
  unsigned refuse_disable{};
  std::array<bool,7> enabled{false,true,true,true,true,true,true},frozen{},respond{},complete{},sync{};
  std::array<unsigned,7> drop_position{};  // liczba kolejnych odpowiedzi 0x31 do pominięcia
  std::array<std::int64_t,7> drive_offset{};  // układ celu F5 napędu przesunięty względem enkodera 0x31
  std::array<std::int64_t,7> disable_settle{};  // osiadanie wału po ENABLE=0 [impulsy 0x31]
  std::array<std::int64_t,7> positions{},targets{};
  // Nastawy z katalogu kopii (klucz: oś<<16 | kod<<8 | sub+1); wartość w formacie zapisu (z podkomendą).
  std::map<std::uint32_t,std::vector<std::uint8_t>> params;unsigned reject_param_write{},unsupported_param{};
  std::array<std::uint8_t,7> motor_state_override{};  // 0 = F1 zgłasza 1 (postój); np. 3 = hamowanie bez końca
  static std::uint32_t param_key(unsigned axis,std::uint8_t code,int sub) {return (axis<<16)|(unsigned(code)<<8)|unsigned(sub+1);}
  std::array<unsigned,256> counts{};
  struct Event {Time stamp;unsigned axis;std::vector<std::uint8_t> data;};
  std::vector<Event> events;
  std::deque<Frame> rx;
  Time now{1s};Clock::duration delay{1ms};
  unsigned reported_bitrate{};
  bool tx_confirmations{true},drop_sync_confirmation{},fail_f5{};
  unsigned missing_ack{},rejected_ack{},late_ack{},bad_firmware{};
  bool missing_speed{};
  bool clear_pending_on_sync_disable{true}, reject_disabled_f5{};
  bool f7_clears_pending{true};
  std::uint32_t watchdog{300};
  std::array<bool,7> pending_stops{};
  std::vector<Time> open_attempts;
  std::array<Clock::duration,7> axis_ack_delay{};
  std::array<unsigned,7> drop_ack{};
  std::array<unsigned,7> speed_blips{};  // N kolejnych odczytów 0x32 zwraca speed_blip_rpm (drgania w trzymaniu)
  std::uint8_t speed_blip_rpm{1};
  // Diagnostyka napędu: prąd [mA] (FFFF = bez odczytu), PID vFOC Kp/Ki/Kd/Kv (domyślne MKS), błąd kąta 39h.
  std::uint16_t work_current_ma{1600};std::array<std::uint16_t,4> pid{0xdc,0x64,0x10e,0x140};
  bool pid_without_sub{},silent_pid{};std::array<std::int32_t,7> angle_error{};
  std::array<std::uint8_t,7> microsteps{16,16,16,16,16,16,16};  // gubi N kolejnych ACK F5/05 danej osi (F5 trafia do bufora)
  Clock::duration sync_echo_delay{0ms};
  std::array<std::int64_t,7> pending_targets{};
  std::array<bool,7> pending_targets_valid{};
  unsigned activated_sets{};
  bool supports_tx_confirmation() const override {return tx_confirmations;}
  unsigned bitrate() const override {return reported_bitrate;}
  void open(const std::string &) override {
    open_attempts.push_back(now);
    if(disconnected) throw std::runtime_error("USB odłączone");
    opened=true;
  }
  void close() override {opened=false;}
  bool is_open() const override {return opened;}
  void send(std::uint16_t axis,const std::vector<std::uint8_t> &data) override {
    if(disconnected) throw std::runtime_error("USB odłączone");
    ++counts[data[0]];events.push_back({now,axis,data});
    if(data[0]==0xf5 && fail_f5 && axis==3) throw std::runtime_error("Błąd wysyłki trzeciej osi");
    if(axis==0 && data[0]==0x4b) {
      for(unsigned j=1;j<=6;++j) if(pending_targets_valid[j]) {
        if(!pending_stops[j]) {targets[j]=pending_targets[j];if(!frozen[j]) positions[j]=targets[j]+drive_offset[j];}
        pending_targets_valid[j]=false;pending_stops[j]=false;
      }
      ++activated_sets;
      if(!drop_sync_confirmation) rx.push_back({0,data,false,0,now+sync_echo_delay,0,true});
    }
    if(echo) rx.push_back({axis,data,false,0,now+delay});
    if(!axis || axis>6 || axis==absent || axis==extra_absent) return;
    std::vector<std::uint8_t> reply;
    switch(data[0]) {
      case 0x40:reply={0x40,0x12,1,0,static_cast<uint8_t>(bad_firmware ? 7 : 9),0x20};break;
      case 0:
        if(data.size()>=3) {
          const int s=(data[1]==0x96 || data[1]==0x97) && data.size()==4 ? data[2] : -1;
          const auto it=params.find(param_key(axis,data[1],s));
          if(it!=params.end()) {reply={data[1]};reply.insert(reply.end(),it->second.begin(),it->second.end());break;}
          if(unsupported_param==data[1]) {reply={data[1],0xff,0xff};break;}  // firmware bez odczytu tej nastawy
        }
        if(data[1]==0x8c) reply={0x8c,static_cast<uint8_t>(bad_response_mode || respond[axis]),static_cast<uint8_t>(complete[axis])};
        else if(data[1]==0x83) reply={0x83,static_cast<uint8_t>(work_current_ma>>8),static_cast<uint8_t>(work_current_ma)};
        else if(data[1]==0x96) {
          if(silent_pid) return;
          const auto a=pid[data[2]*2],b=pid[data[2]*2+1];
          reply={0x96};if(!pid_without_sub) reply.push_back(data[2]);
          for(auto v:{a,b}) {reply.push_back(static_cast<uint8_t>(v>>8));reply.push_back(static_cast<uint8_t>(v));}
        }
        else if(data[1]==0x98) {
          const auto value=bad_watchdog ? 256u : watchdog;
          reply={0x98,static_cast<uint8_t>(value>>24),static_cast<uint8_t>(value>>16),static_cast<uint8_t>(value>>8),static_cast<uint8_t>(value)};
        }
        else reply={data[1],static_cast<uint8_t>(data[1]==0x82 ? 5 : data[1]==0x84 ? microsteps[axis] : bad_sync_mode || sync[axis])};
        break;
      case 0x8c:respond[axis]=data[1];complete[axis]=data[2];reply={0x8c,1};break;
      case 0x4a:sync[axis]=data[1];if(!sync[axis] && clear_pending_on_sync_disable) pending_targets_valid[axis]=false;reply={0x4a,1};break;
      case 0xf7:if(f7_clears_pending) pending_targets_valid[axis]=false;reply={data[0],1};break;
      case 0x82:case 0x98:reply={data[0],1};break;
      case 0x84:microsteps[axis]=data[1];reply={0x84,1};break;
      case 0xf3:if(!data[1] && enabled[axis]) positions[axis]+=disable_settle[axis];
                if(data[1] || axis!=refuse_disable) enabled[axis]=data[1];reply={0xf3,static_cast<uint8_t>(axis==refuse_disable ? 0 : 1)};break;
      case 0x92:positions[axis]=0;reply={0x92,1};break;
      case 0x83:case 0x9b:case 0x85:case 0x86:case 0x87:case 0x88:case 0x89:case 0x8d:case 0x8f:case 0x95:case 0x96:case 0x97:case 0x9d:case 0x9f:case 0x42: {
        const int s=(data[0]==0x96 || data[0]==0x97) ? data[1] : -1;
        const auto key=param_key(axis,data[0],s);
        if(reject_param_write==data[0]) {reply={data[0],0};break;}
        params[key]=std::vector<std::uint8_t>(data.begin()+1,data.end()-1);reply={data[0],1};break;
      }
      case 0x3a:reply={0x3a,static_cast<uint8_t>(enabled[axis])};break;
      case 0x3e:reply={0x3e,static_cast<uint8_t>(stalled==axis)};break;
      case 0x32:if(missing_speed) return;reply={0x32,0,static_cast<std::uint8_t>(speed_blips[axis] && speed_blips[axis]-- ? speed_blip_rpm : 0)};break;
      case 0xf1:reply={0xf1,static_cast<std::uint8_t>(motor_state_override[axis] ? motor_state_override[axis] : 1)};break;
      case 0x39:{const auto v=static_cast<std::uint32_t>(angle_error[axis]);reply={0x39,static_cast<uint8_t>(v>>24),static_cast<uint8_t>(v>>16),static_cast<uint8_t>(v>>8),static_cast<uint8_t>(v)};break;}
      case 0x31: {
        if(axis==missing_position) return;
        if(drop_position[axis]) {--drop_position[axis];return;}
        reply={0x31};const auto value=static_cast<std::uint64_t>(positions[axis]);
        for(int shift=40;shift>=0;shift-=8) reply.push_back(static_cast<std::uint8_t>(value>>shift));break;
      }
      case 0xf5: {
        const auto raw=(std::uint32_t(data[4])<<16)|(std::uint32_t(data[5])<<8)|data[6];
        const bool stop=data[1]==0 && data[2]==0 && data[3]==0 && raw==0;
        const bool rejected=axis==rejected_ack || (reject_disabled_f5 && !enabled[axis]);
        if(sync[axis]) {
          if(!rejected) {
            pending_targets[axis]=raw&0x800000 ? static_cast<std::int32_t>(raw)-0x1000000 : static_cast<std::int32_t>(raw);
            pending_targets_valid[axis]=true;pending_stops[axis]=stop;
          }
          if(axis==missing_ack || (drop_ack[axis] && drop_ack[axis]--)) return;
          reply={0xf5,static_cast<uint8_t>(rejected ? 0 : 5)};
          reply.push_back(checksum(axis,reply));
          rx.push_back({axis,reply,false,0,now+(axis==late_ack ? 20ms : axis_ack_delay[axis]!=Clock::duration{} ? axis_ack_delay[axis] : delay)});return;
        }
        if(stop || rejected) return;
        targets[axis]=raw&0x800000 ? static_cast<std::int32_t>(raw)-0x1000000 : static_cast<std::int32_t>(raw);
        if(!frozen[axis]) positions[axis]=targets[axis]+drive_offset[axis];return; // brak jakiegokolwiek ACK
      }
      case 0xf6:case 0xfe:return;
      default:return;
    }
    reply.push_back(checksum(axis,reply));rx.push_back({axis,reply,false,0,now+delay});
  }
  std::optional<Frame> receive() override {
    if(disconnected) throw std::runtime_error("USB odłączone");
    const auto it=std::find_if(rx.begin(),rx.end(),[&](const auto &f){return f.received_at<=now;});
    if(it==rx.end()) return {};
    auto frame=*it;rx.erase(it);return frame;
  }
};
inline BusConfig config(unsigned bitrate=1000000) {
  BusConfig c;c.bitrate=bitrate;c.program_enabled=true;c.program_revision="bench";
  for(unsigned j=0;j<6;++j) c.axes[j]={static_cast<uint16_t>(j+1),10,1,0,-2,2,2,100,.2,240};
  return c;
}
struct Rig {
  Port *port;std::unique_ptr<CanBusManager> bus;Time now{1s};
  explicit Rig(BusConfig c=config()) {
    auto p=std::make_unique<Port>();port=p.get();bus=std::make_unique<CanBusManager>(c,std::move(p));
    bus->open(now);bus->require_configuration();RobotProfile profile;profile.revision="bench";profile.changed.fill(true);
    for(unsigned j=0;j<6;++j) {const auto &a=c.axes[j];profile.axes[j]={true,a.gear_ratio,a.direction,a.offset,a.lower,a.upper,a.max_velocity,a.max_acceleration};}
    if(!bus->configure(ConfigurationPhase::bootstrap,"boot","",profile,now).accepted || !bus->configure(ConfigurationPhase::finish,"boot","",profile,now).accepted) throw std::runtime_error("bootstrap");
    bus->configure_program_authorization(c);
  }
  void run(unsigned ms) {for(unsigned i=0;i<ms;++i) {port->now=now;bus->tick(now);now+=1ms;}}
  void ready() {run(800);if(bus->snapshot().scanning) throw std::runtime_error("scan");}
  void enabled_for_settings(bool enabled) {port->enabled.fill(enabled);run(150);}
  void synchronized() {
    enabled_for_settings(false);
    auto p=bus->configure(ConfigurationPhase::status,"","",{},now).resolved_configuration;
    p.revision="sync-bench";p.ptp_lin_dispatch_mode="synchronized";p.changed.fill(false);
    if(!bus->configure(ConfigurationPhase::prepare,"sync-settings","bench",p,now).accepted ||
        !bus->configure(ConfigurationPhase::commit,"sync-settings","bench",p,now).accepted ||
        !bus->configure(ConfigurationPhase::finish,"sync-settings","bench",p,now).accepted) throw std::runtime_error("sync-settings");
    enabled_for_settings(true);
  }
  PreparedMotionProgram program(double target=.2,double seconds=1,bool partial=false) {
    bus->update_motion_model("model");bus->update_motion_settings("settings",partial,now);
    PreparedMotionProgram p;p.id="plan";p.model_id="model";p.settings_revision="settings";
    p.revision=bus->configure(ConfigurationPhase::status,"","",{},now).revision;
    ProgramPoint a,b;b.seconds=seconds;const auto state=bus->control_state();
    for(unsigned j=0;j<6;++j) {a.positions[j]=state.positions[j].value;b.positions[j]=target;}
    p.points={a,b};return p;
  }
  void execute(PreparedMotionProgram p,const std::string &key="run") {
    const auto status=bus->prepare_program(std::move(p),now);
    if(!bus->execute_program(status.program_id,key,now)) throw std::runtime_error("execute");
  }
};
}
