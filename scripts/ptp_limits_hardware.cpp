// Stanowisko jednego rozsprzęglonego wału. Nie jest konfiguracją robota.
// Budować przez build_ptp_limits.py; jawne --execute i zaliczony STOP są wymagane.
#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include "mks_servo_can/protocol.hpp"
#include "mks_servo_can/trace.hpp"
#include <algorithm>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

using namespace mks_servo_can;
using namespace std::chrono_literals;
namespace {
constexpr double pi=3.14159265358979323846;
volatile std::sig_atomic_t interrupted=0;
void signal_handler(int) { interrupted=1; }
double seconds(Time a,Time b) { return std::chrono::duration<double>(a-b).count(); }
void note(const std::string &event,const std::string &detail="") {
  std::cout << "{\"event\":" << std::quoted(event) << ",\"mono_ns\":" << trace_ns(Clock::now())
            << ",\"detail\":" << std::quoted(detail) << "}" << std::endl;
}
class ShaftTransport final : public Transport {
 public:
  SocketCan socket;
  Time first_sync{}, last_position{}, last_speed{};
  double position{}, maximum=-1e9, minimum=1e9;
  int speed{};
  unsigned sync_count{};
  void open(const std::string &name) override { socket.open(name); }
  void close() override { socket.close(); }
  bool is_open() const override { return socket.is_open(); }
  void send(std::uint16_t address,const std::vector<std::uint8_t> &data) override {
    if(data.empty()) throw std::runtime_error("Pusta ramka");
    if(data[0]==0xf5) {
      const unsigned rpm=(unsigned(data.at(1))<<8)|data.at(2);
      const int raw=(int(data.at(4))<<16)|(int(data.at(5))<<8)|data.at(6);
      if(address!=1 || rpm>1 || raw<0 || raw>std::lround(10.*16384/360))
        throw std::runtime_error("Rozkaz poza zatwierdzonym CAN 1 / 10 stopni / 1 RPM");
    }
    if(data[0]==0xf6 && (data.at(1)!=0 || data.at(2)!=0)) throw std::runtime_error("Zakaz niezerowego F6");
    if(data[0]==0xf3 && data.at(1) && address!=1) throw std::runtime_error("Zakaz ENABLE innych osi");
    // Celowo izolowany uczestnik: manager nie odkrywa ani nie odpytuje innych osi.
    // STOP nadal przechodzi do wszystkich adresów 1–6.
    if(address!=0 && address!=1 && data[0]!=0xf7 && data[0]!=0xf6) return;
    socket.send(address,data);
    if(address==0 && data[0]==0x4b) {
      if(!sync_count++) first_sync=Clock::now();
    }
  }
  std::optional<Frame> receive() override {
    for(unsigned n=0;n<64;++n) {
      auto frame=socket.receive();if(!frame) return {};
      if(frame->error) throw std::runtime_error("Ramka bledu CAN na stanowisku");
      if(frame->address!=1) continue;
      if(frame->data.size()==8 && frame->data[0]==0x31) {
        if(auto value=parse_position(1,frame->data)) {
          position=*value*360./16384;last_position=frame->received_at;
          if(sync_count) { maximum=std::max(maximum,position);minimum=std::min(minimum,position); }
        }
      }
      if(frame->data.size()==4 && frame->data[0]==0x32) {
        if(auto value=parse_speed(1,frame->data)) {speed=*value;last_speed=frame->received_at;}
      }
      return frame;
    }
    return {};
  }
};
class Run {
 public:
  std::unique_ptr<CanBusManager> manager;
  ShaftTransport *transport{};
  Time due=Clock::now(), previous{};
  double max_gap{};
  explicit Run() {
    BusConfig config;
    config.feedback_timeout=std::chrono::milliseconds(PTP_BENCH_FEEDBACK_MS);
    config.response_timeout=std::chrono::milliseconds(PTP_BENCH_RESPONSE_MS);
    // Współrzędna stanowiskowa oznacza radiany WAŁU; nie zastępuje geometrii J1.
    for(unsigned j=0;j<6;++j)
      config.axes[j]={static_cast<uint16_t>(j+1),1,1,0,-pi,pi,2*pi/60,2*pi/(60*.01275),1.2*pi/180,1};
    auto bus=std::make_unique<ShaftTransport>();transport=bus.get();
    manager=std::make_unique<CanBusManager>(config,std::move(bus),Clock::now);
    manager->open(Clock::now());
    if(!manager->snapshot().open) throw std::runtime_error(manager->snapshot().message);
    manager->start_scan();
  }
  void tick(bool allow_fault=false) {
    if(interrupted) throw std::runtime_error("Przerwanie operatora");
    const auto begin=Clock::now();
    if(previous!=Time{}) max_gap=std::max(max_gap,seconds(begin,previous)*1000);
    previous=begin;manager->tick(begin);
    const auto end=Clock::now();
    if(seconds(end,begin)>.002) { TraceEvent e;e.event="io_tick_slow";e.duration_ns=trace_ns(end)-trace_ns(begin);trace(e); }
    if(!allow_fault && manager->snapshot().fault) throw std::runtime_error(manager->snapshot().message);
    due+=1ms;if(due<end) due=end;
    std::this_thread::sleep_until(due);
  }
  template<class Predicate> void wait(Predicate predicate,double timeout,const std::string &label) {
    const auto deadline=Clock::now()+std::chrono::duration<double>(timeout);
    while(!predicate()) { if(Clock::now()>=deadline) throw std::runtime_error(label);tick(); }
  }
  void run() {
    wait([&]{return !manager->snapshot().scanning;},3,"Skan stanowiska");
    if(!manager->manual().set_enable(1,true,Clock::now())) throw std::runtime_error("Odrzucono ENABLE 1");
    wait([&]{return !manager->manual().busy();},3,"ENABLE 1");
    wait([&]{return manager->manual().snapshot(Clock::now()).axes[0].can_zero;},3,"Brak stabilnego postoju przed zerowaniem");
    if(!manager->manual().zero(1,Clock::now())) throw std::runtime_error("Odrzucono funkcje zeruj wal");
    wait([&]{return !manager->manual().busy();},3,"Zerowanie");
    note("shaft_zero",manager->manual().snapshot(Clock::now()).message);
    // Jawne odniesienie stanowiskowe powstaje po zerowaniu; nie zapisuje profilu ROS.
    manager->require_configuration();
    RobotProfile profile;profile.revision="shaft-bench";profile.changed[0]=true;
    profile.axes[0]={true,1,1,0,-pi,pi,2*pi/60,2*pi/(60*.01275)};
    if(!manager->configure(ConfigurationPhase::bootstrap,"shaft-boot","",profile,Clock::now()).accepted ||
       !manager->configure(ConfigurationPhase::finish,"shaft-boot","",profile,Clock::now()).accepted)
      throw std::runtime_error("Profil stanowiska");
    wait([&]{return !manager->snapshot().scanning;},3,"Skan po bootstrap");
    const auto warm=Clock::now();while(Clock::now()-warm<200ms) tick();
    BusConfig authorization=manager->config();authorization.program_enabled=true;
    authorization.program_revision="shaft-bench";
    // Dotyczy wyłącznie procesu stanowiskowego po osobnym teście F7→F6(0).
    authorization.program_stop_verified=authorization.program_sync_verified=authorization.program_staged_reset_verified=true;
    manager->configure_program_authorization(authorization);
    manager->update_motion_model("isolated-shaft-1");manager->update_motion_settings("bench-settings",true,Clock::now());
    PreparedMotionProgram program;program.id="shaft-10deg";program.revision="shaft-bench";
    program.model_id="isolated-shaft-1";program.settings_revision="bench-settings";
    // Quintic 3,2 s: szczyt 0,977 RPM, cel 10°, siatka 20 ms.
    for(unsigned k=0;k<=160;++k) {
      const double t=k*.02,u=t/3.2,angle=10*pi/180;
      ProgramPoint point;point.seconds=t;
      point.positions[0]=angle*(10*std::pow(u,3)-15*std::pow(u,4)+6*std::pow(u,5));
      point.velocities[0]=angle*(30*u*u-60*std::pow(u,3)+30*std::pow(u,4))/3.2;
      point.accelerations[0]=angle*(60*u-180*u*u+120*std::pow(u,3))/(3.2*3.2);
      program.points.push_back(point);
    }
    const auto status=manager->prepare_program(std::move(program),Clock::now());
    note("prepared",status.mode);
    if(!manager->execute_program(status.program_id,"shaft-trial",Clock::now())) throw std::runtime_error("Odrzucono start");
    const auto began=Clock::now();bool speed_checked=false;
    while(manager->program_status().busy) {
      if(seconds(Clock::now(),began)>12) throw std::runtime_error("Globalny termin stanowiska 12 s");
      tick();
      if(transport->sync_count && !speed_checked && transport->last_speed>=transport->first_sync+3300ms) {
        speed_checked=true;
        std::cout<<"{\"event\":\"speed_after_100ms\",\"mono_ns\":"<<trace_ns(transport->last_speed)
                 <<",\"rpm\":"<<transport->speed<<",\"position_degrees\":"<<transport->position<<"}"<<std::endl;
      }
    }
    note("program_result",manager->program_status().state+": "+manager->program_status().message);
    if(manager->program_status().state!="succeeded") throw std::runtime_error(manager->program_status().message);
    if(!speed_checked || transport->speed!=0 || std::abs(transport->position-10)>.3)
      throw std::runtime_error("Koncowe kryteria stanowiska niepotwierdzone");
  }
  void summary(bool success,const std::string &message) {
    const auto s=manager->snapshot().statistics;
    std::cout<<"{\"event\":\"trial_result\",\"success\":"<<(success?"true":"false")
      <<",\"message\":"<<std::quoted(message)<<",\"sync_count\":"<<transport->sync_count
      <<",\"final_degrees\":"<<transport->position<<",\"speed_rpm\":"<<transport->speed
      <<",\"max_degrees\":"<<(transport->sync_count?transport->maximum:transport->position)
      <<",\"sync_lateness_max_ms\":"<<s.sync_lateness_ms.maximum<<",\"ack_margin_min_ms\":"<<s.ack_margin_ms.minimum
      <<",\"rtt_max_ms\":"<<s.rtt.maximum<<",\"io_gap_max_ms\":"<<max_gap
      <<",\"timeouts\":"<<s.timeouts<<",\"peak_utilization\":"<<s.peak_scheduled_utilization<<"}"<<std::endl;
  }
  void cleanup() {
    manager->stop("Koniec proby stanowiskowej");
    const auto until=Clock::now()+60ms;
    while(manager->snapshot().statistics.stop_pending && Clock::now()<until) tick(true);
    // Wyłączenie CAN 1 także po błędzie, bez nowych SYNC.
    transport->socket.send(1,enable_request(1,false));
    manager->close();
  }
};
}
int main(int argc,char **argv) {
  if(argc!=3 || std::string(argv[1])!="--execute" || std::string(argv[2])!="--stop-clear-verified") {
    std::cerr<<"Wymagane --execute --stop-clear-verified po zgodzie operatora i osobnym odbiorze STOP\n";return 2;
  }
  std::signal(SIGINT,signal_handler);std::signal(SIGTERM,signal_handler);
  try {
    Run run;bool passed=false;std::string message;
    try {run.run();passed=true;message="PTP i kryteria koncowe potwierdzone";}
    catch(const std::exception &e) {message=e.what();note("failure",message);}
    run.summary(passed,message);
    try {interrupted=0;run.cleanup();} catch(const std::exception &e) {note("cleanup_error",e.what());passed=false;}
    return passed?0:1;
  } catch(const std::exception &e) {note("startup_error",e.what());return 2;}
}
