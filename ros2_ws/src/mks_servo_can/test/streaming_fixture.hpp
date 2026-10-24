#pragma once
#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include <algorithm>
#include <deque>
#include <stdexcept>
using namespace mks_servo_can;
using namespace std::chrono_literals;
namespace streaming_test {
class Port final : public Transport {
 public:
  bool opened{},disconnected{},echo{},bad_response_mode{},bad_sync_mode{},bad_watchdog{};
  unsigned absent{},extra_absent{},missing_position{},stalled{};
  std::array<bool,7> enabled{false,true,true,true,true,true,true},frozen{},respond{},complete{},sync{};
  std::array<std::int64_t,7> positions{},targets{};
  std::array<unsigned,256> counts{};
  struct Event {Time stamp;unsigned axis;std::vector<std::uint8_t> data;};
  std::vector<Event> events;
  std::deque<Frame> rx;
  Time now{1s};Clock::duration delay{1ms};
  unsigned reported_bitrate{};
  unsigned bitrate() const override {return reported_bitrate;}
  void open(const std::string &) override {opened=true;}
  void close() override {opened=false;}
  bool is_open() const override {return opened;}
  void send(std::uint16_t axis,const std::vector<std::uint8_t> &data) override {
    if(disconnected) throw std::runtime_error("USB odłączone");
    ++counts[data[0]];events.push_back({now,axis,data});
    if(echo) rx.push_back({axis,data,false,0,now+delay});
    if(!axis || axis>6 || axis==absent || axis==extra_absent) return;
    std::vector<std::uint8_t> reply;
    switch(data[0]) {
      case 0x40:reply={0x40,0x12,1,0,9,0x20};break;
      case 0:
        if(data[1]==0x8c) reply={0x8c,static_cast<uint8_t>(bad_response_mode || respond[axis]),static_cast<uint8_t>(complete[axis])};
        else if(data[1]==0x98) reply={0x98,0,0,1,static_cast<uint8_t>(bad_watchdog ? 0 : 44)};
        else reply={data[1],static_cast<uint8_t>(data[1]==0x82 ? 5 : data[1]==0x84 ? 16 : bad_sync_mode || sync[axis])};
        break;
      case 0x8c:respond[axis]=data[1];complete[axis]=data[2];reply={0x8c,1};break;
      case 0x4a:sync[axis]=data[1];reply={0x4a,1};break;
      case 0x82:case 0x98:case 0xf7:reply={data[0],1};break;
      case 0xf3:enabled[axis]=data[1];reply={0xf3,1};break;
      case 0x92:positions[axis]=0;reply={0x92,1};break;
      case 0x3a:reply={0x3a,static_cast<uint8_t>(enabled[axis])};break;
      case 0x3e:reply={0x3e,static_cast<uint8_t>(stalled==axis)};break;
      case 0x32:reply={0x32,0,0};break;
      case 0xf1:reply={0xf1,1};break;
      case 0x31: {
        if(axis==missing_position) return;
        reply={0x31};const auto value=static_cast<std::uint64_t>(positions[axis]);
        for(int shift=40;shift>=0;shift-=8) reply.push_back(static_cast<std::uint8_t>(value>>shift));break;
      }
      case 0xf5: {
        const auto raw=(std::uint32_t(data[4])<<16)|(std::uint32_t(data[5])<<8)|data[6];
        targets[axis]=raw&0x800000 ? static_cast<std::int32_t>(raw)-0x1000000 : raw;
        if(!frozen[axis]) positions[axis]=targets[axis];return; // brak jakiegokolwiek ACK
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
  PreparedMotionProgram program(double target=.2,double seconds=1,bool partial=false) {
    bus->update_motion_model("model");bus->update_motion_settings("settings",partial,now);
    PreparedMotionProgram p;p.id="plan";p.model_id="model";p.settings_revision="settings";p.revision="bench";
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
