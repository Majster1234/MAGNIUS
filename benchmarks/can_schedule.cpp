#include "harness.hpp"
#include <thread>
#include <cstdlib>
#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include <algorithm>
#include <deque>
#include <map>
#include <linux/can/error.h>
using namespace mks_servo_can;
using namespace std::chrono_literals;
namespace {
class Emulator final : public Transport {
 public:
  bool partial{false};
  unsigned max_present{6};
  bool opened{false}, disconnected{false}, own_echo{false}, complete_before_ack{false};
  std::uint16_t missing_ack{}, missing_position{};
  std::uint16_t absent_address{};
  std::array<bool,21> enabled{}, sync{};
  std::array<unsigned, 21> position_requests{};
  Time now{};
  std::array<std::uint64_t, 256> counts{};
  std::array<std::int64_t, 21> positions{}, prepared{};
  std::map<std::uint16_t, std::vector<std::uint8_t>> prepared_payloads;
  std::deque<Frame> responses;
  std::vector<std::pair<std::uint16_t, std::uint8_t>> trace;
  void open(const std::string &) override { opened = true; }
  void close() override { opened = false; }
  bool is_open() const override { return opened; }
  void send(std::uint16_t address, const std::vector<std::uint8_t> &data) override {
    if (disconnected) throw std::runtime_error("USB odłączone");
    ++counts[data[0]];
    if (trace.size() < 10000) trace.emplace_back(address, data[0]);
    if (own_echo) responses.push_back({address, data, false, 0, now + 1ms});
    if (address == 0 && data[0] == 0x4b) { positions = prepared; return; }
    if (address < 1 || address > max_present) return;
    if (data[0] == 0x31) ++position_requests[address];
    if (address == absent_address || (partial && (address==4 || address==6))) return;
    std::vector<std::uint8_t> reply;
    switch (data[0]) {
      case 0x40: reply = {0x40, 0x12, 1, 0, 9, 0x20}; break;
      case 0x00:
        if(data[1]==0x98) reply={0x98,0,0,1,44};
        else reply={data[1],static_cast<uint8_t>(data[1]==0x82 ? 5 : data[1]==0x84 ? 16 : sync[address])};
        break;
      case 0x4a: sync[address]=data[1];reply={0x4a,1};break;
      case 0x82: case 0x8c: case 0x98: case 0xf7: reply={data[0],1};break;
      case 0xf6: reply={0xf6,1};break;
      case 0xf3: enabled[address]=data[1];reply={0xf3,1};break;
      case 0x32: reply={0x32,0,0};break;
      case 0x92: positions[address]=0;reply={0x92,1};break;
      case 0x3a: reply = {0x3a, static_cast<uint8_t>(enabled[address])}; break;
      case 0x3e: reply = {0x3e, 0}; break;
      case 0xf1: reply = {0xf1, 1}; break;
      case 0x31: {
        if (missing_position == address) return;
        reply = {0x31};
        const auto p = static_cast<std::uint64_t>(positions[address]);
        for (int shift = 40; shift >= 0; shift -= 8) reply.push_back(static_cast<std::uint8_t>(p >> shift));
        break;
      }
      case 0xf5: {
        prepared_payloads[address]=data;
        const auto raw = (std::int32_t(data[4]) << 16) | (std::int32_t(data[5]) << 8) | data[6];
        prepared[address] = raw & 0x800000 ? raw - 0x1000000 : raw;
        if (missing_ack == address) return;
        if(complete_before_ack) {
          auto complete=std::vector<uint8_t>{0xf5,2};complete.push_back(checksum(address,complete));
          responses.push_back({address,complete,false,0,now+1ms});
        }
        reply = {0xf5, 5}; break;
      }
      default: return;
    }
    reply.push_back(checksum(address, reply));
    responses.push_back({address, reply, false, 0, now + 1ms});
  }
  std::optional<Frame> receive() override {
    if (disconnected) throw std::runtime_error("USB odłączone");
    if (responses.empty() || responses.front().received_at > now) return std::nullopt;
    auto frame = responses.front(); responses.pop_front(); return frame;
  }
};
BusConfig configured() {
  BusConfig config; config.motion_enabled = true;
  config.commissioning_verified = config.synchronization_verified = config.stop_verified = true;
  for (std::size_t i = 0; i < kAxes; ++i)
    config.axes[i] = {static_cast<std::uint16_t>(i + 1), 10, 1, 0, -2, 2, 1, 100, 0.2, 1};
  return config;
}
struct Harness {
  Emulator *bus;
  std::unique_ptr<CanBusManager> manager;
  Time now{1s};
  explicit Harness(BusConfig config = configured()) {
    auto transport = std::make_unique<Emulator>(); bus = transport.get(); bus->now = now;
    manager = std::make_unique<CanBusManager>(config, std::move(transport));
    manager->open(now);
    manager->require_configuration();
    RobotProfile profile;profile.revision="bench";profile.changed.fill(true);
    for(auto &a:profile.axes) a={true,10,1,0,-2,2,1,100};
    if(!manager->configure(ConfigurationPhase::bootstrap,"boot","",profile,now).accepted ||
       !manager->configure(ConfigurationPhase::finish,"boot","",profile,now).accepted) throw std::runtime_error("bootstrap");
    config.program_enabled=true;config.program_revision="bench";
    config.program_stop_verified=config.program_sync_verified=config.program_staged_reset_verified=true;
    manager->configure_program_authorization(config);
  }
  void run(int ms) {
    for (int i = 0; i < ms; ++i) { bus->now = now; manager->tick(now); now += 1ms; }
  }
  void ready() { run(45000); if(manager->snapshot().scanning) throw std::runtime_error("scan"); }
  CommandBatch batch(std::uint64_t sequence, double position = 0.001) const {
    CommandBatch batch; batch.positions.fill(position); batch.sequence = sequence;
    batch.created = now; batch.expires = now + 19ms; batch.source = Source::trajectory; return batch;
  }
};
}

struct Cycle {
  Harness h;
  unsigned kind;
  std::uint64_t count{};
  explicit Cycle(unsigned scenario):kind(scenario) {
    h.bus->partial=kind==1;h.bus->max_present=kind==3 ? 20 : 6;
    if(kind==3) { h.run(45000);h.manager->close();h.manager->open(h.now);h.run(200); }
    else h.ready();
    if(kind<3 || kind==4) enable();
  }
  void enable() {
    for(unsigned j=1;j<=6;++j) if(!(h.bus->partial && (j==4 || j==6))) {
      if(!h.manager->manual().set_enable(j,true,h.now)) throw std::runtime_error("enable refused");
      h.run(200);
      if(!h.manager->manual().prepared(j)) throw std::runtime_error("enable failed");
    }
  }
  void cycle(std::uint64_t) {
    if(kind==3) { h.run(20); return; }
    h.manager->update_motion_model("model");
    h.manager->update_motion_settings("settings",kind==1,h.now);
    PreparedMotionProgram p;p.id="bench";p.model_id="model";p.revision="bench";p.settings_revision="settings";
    ProgramPoint a,b;b.seconds=(kind<2 ? 2. : .2);p.points={a,b};
    if(kind<2) { auto middle=a;middle.seconds=1;middle.positions.fill(.1);p.points.insert(p.points.begin()+1,middle); }
    auto s=h.manager->prepare_program(std::move(p),h.now);
    if(!h.manager->execute_program(s.program_id,"key"+std::to_string(++count),h.now)) throw std::runtime_error("execute");
    if(kind==4) {
      h.run(80);h.manager->stop("benchmark");h.run(50);h.manager->close();
      // API dostępne po obu stronach pomiaru; ścieżka zachowania adresów jest badana osobno w testach.
      h.manager->reset_session();h.manager->open(h.now);h.manager->start_scan();h.ready();enable();
    } else {
      h.run(kind<2 ? 2500 : 800);
      if(h.manager->program_status().state!="succeeded") throw std::runtime_error(h.manager->program_status().message);
    }
  }
  bool verify() const { return !h.manager->snapshot().fault; }
};
int main(int argc,char **argv) {
  const std::string executable=argv[0];
  unsigned kind=executable.find("stream4")!=std::string::npos ? 1 : executable.find("native")!=std::string::npos ? 2 :
      executable.find("idle20")!=std::string::npos ? 3 : executable.find("reset")!=std::string::npos ? 4 : 0;
  if(const auto duration=std::getenv("CAN_BENCH_REALTIME_SECONDS")) {
    Cycle cycle(3); const auto start=Clock::now();auto due=start;
    const auto seconds=std::stod(duration);
    while(std::chrono::duration<double>(Clock::now()-start).count()<seconds) {
      cycle.cycle(0);due+=20ms;std::this_thread::sleep_until(due);
    }
    return cycle.verify() ? 0 : 1;
  }
  return robot_benchmarks::main(argc,argv,[kind]{return Cycle(kind);});
}
