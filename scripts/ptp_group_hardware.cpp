// Próba fizycznego PTP: wyłącznie po zgodzie operatora, wały rozsprzęglone.
// Korzysta z produkcyjnej biblioteki i jawnych parametrów próby.
#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include "mks_servo_can/trace.hpp"
#include <algorithm>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <linux/can/error.h>
using namespace mks_servo_can;
using namespace std::chrono_literals;
namespace {
constexpr double pi=3.14159265358979323846;
volatile std::sig_atomic_t interrupted=0;
void signal_handler(int) { interrupted=1; }
void note(const std::string &event,const std::string &detail="") {
  std::cout<<"{\"event\":"<<std::quoted(event)<<",\"mono_ns\":"<<trace_ns(Clock::now())
           <<",\"utc_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count()
           <<",\"detail\":"<<std::quoted(detail)<<"}"<<std::endl;
}
struct Options {
  std::array<bool,6> selected{};
  std::array<AxisConfig,6> axes{};
  std::string profile,axis_list;
  unsigned tick_us{1000};
  double duration_scale{1};
  unsigned ack_ms{10};
  double tolerance_deg{.5};
};
Options options(int argc,char **argv) {
  if(argc<7 || std::string(argv[1])!="--execute" || std::string(argv[2])!="--decoupled")
    throw std::invalid_argument("Wymagane --execute --decoupled --axes 1|1,2|1,2,3,5 --profile PLIK");
  Options out;
  for(int i=3;i<argc;i+=2) {
    if(i+1>=argc) throw std::invalid_argument("Brak wartości argumentu");
    const std::string key=argv[i],value=argv[i+1];
    if(key=="--axes") out.axis_list=value;
    else if(key=="--profile") out.profile=value;
    else if(key=="--tick-us") out.tick_us=std::stoul(value);
    else if(key=="--duration-scale") out.duration_scale=std::stod(value);
    else if(key=="--ack-ms") out.ack_ms=std::stoul(value);
    else if(key=="--tolerance-deg") out.tolerance_deg=std::stod(value);
    else throw std::invalid_argument("Nieznany argument");
  }
  if(out.axis_list!="1" && out.axis_list!="1,2" && out.axis_list!="1,2,3,5") throw std::invalid_argument("Niezatwierdzona grupa osi");
  if(out.tick_us<200 || out.tick_us>1000 || out.duration_scale<1 || out.duration_scale>2 || !std::isfinite(out.duration_scale))
    throw std::invalid_argument("Parametry stanowiska poza zakresem");
  if((out.ack_ms!=5 && out.ack_ms!=10) || !std::isfinite(out.tolerance_deg) || out.tolerance_deg<.25 || out.tolerance_deg>1)
    throw std::invalid_argument("ACK/tolerancja poza zakresem zatwierdzonej serii");
  for(char c:out.axis_list) if(c!=',') out.selected[c-'1']=true;
  std::ifstream file(out.profile);
  for(unsigned j=0;j<6;++j) {
    unsigned id,acc;auto &a=out.axes[j];
    if(!(file>>id>>a.gear_ratio>>a.direction>>a.offset>>a.lower>>a.upper>>a.max_velocity>>a.max_acceleration>>acc) || id!=j+1 || acc!=1)
      throw std::invalid_argument("Niepełny profil lub niepotwierdzony acc=1");
    a.address=id;a.acceleration_code=acc;a.tracking_error=kProgramPositionTolerance;
    if(!std::isfinite(a.gear_ratio) || a.gear_ratio<=0 || a.direction!=1 || a.offset!=0 || a.lower>-.5*pi/180 || a.upper<20.5*pi/180)
      throw std::invalid_argument("Profil nie obejmuje zatwierdzonego zakresu po zerowaniu");
    a.max_velocity=std::min(a.max_velocity,2*pi/(60*a.gear_ratio));
  }
  return out;
}
class GuardedTransport final : public Transport {
 public:
  explicit GuardedTransport(const Options &o):options_(o) {}
  SocketCan socket;
  std::array<double,6> angle{},peak{},low{};
  std::array<std::int16_t,6> rpm{};
  std::array<Time,6> stamp{};
  std::array<std::uint16_t,6> microstep{};
  unsigned syncs{};
  void open(const std::string &name) override { socket.open(name); }
  void close() override { socket.close(); }
  bool is_open() const override { return socket.is_open(); }
  unsigned bitrate() const override { return socket.bitrate(); }
  void set_receive_poll_timeout(unsigned value) override { socket.set_receive_poll_timeout(value); }
  void send(std::uint16_t address,const std::vector<std::uint8_t> &data) override {
    if(data.empty()) throw std::runtime_error("Pusta ramka");
    if(data[0]==0xf5) {
      if(!address || address>6 || !options_.selected[address-1]) throw std::runtime_error("F5 do niezatwierdzonej osi");
      const auto j=address-1;const unsigned raw_rpm=(unsigned(data.at(1))<<8)|data.at(2);
      const int raw=(int(data.at(4))<<16)|(int(data.at(5))<<8)|data.at(6);
      const int ticks=(raw&0x800000) ? raw-0x1000000 : raw;
      const auto q=CanBusManager::position_to_joint(options_.axes[j],ticks)*180/pi;
      const auto band=std::max(.5,options_.tolerance_deg);
      if(!microstep[j] || raw_rpm*speed_scale(microstep[j])>1.000000001 || q<-band || q>20+band)
        throw std::runtime_error("F5 poza zakresem 0..20°/1 RPM wału");
    }
    if(data[0]==0xf6 && (data.at(1)!=0 || data.at(2)!=0)) throw std::runtime_error("Zakaz niezerowego F6");
    if(data[0]==0x92) throw std::runtime_error("Zerowanie wykonuje osobno API z utrwaleniem nieważnej kalibracji");
    if(address && (address>6 || !options_.selected[address-1]) && data[0]!=0xf7 && data[0]!=0xf6) return;
    socket.send(address,data);
    if(address==0 && data[0]==0x4b) ++syncs;
  }
  std::optional<Frame> receive() override {
    for(unsigned i=0;i<64;++i) {
      auto f=socket.receive();if(!f) return {};
      if(f->error) {
        const auto &d=f->data;
        const bool recovered=f->error_flags==CAN_ERR_CRTL && d.size()==8 && d[1]==CAN_ERR_CRTL_ACTIVE &&
            !d[0] && !d[2] && !d[3] && !d[4] && !d[5];
        if(recovered) continue;
        throw std::runtime_error("Ramka błędu CAN podczas próby; STOP (flags="+std::to_string(f->error_flags)+")");
      }
      if(!f->address || f->address>6 || !options_.selected[f->address-1]) continue;
      const auto j=f->address-1;
      if(auto p=parse_position(f->address,f->data)) {
        angle[j]=CanBusManager::position_to_joint(options_.axes[j],*p)*180/pi;stamp[j]=f->received_at;
        peak[j]=std::max(peak[j],angle[j]);low[j]=std::min(low[j],angle[j]);
      }
      if(auto v=parse_speed(f->address,f->data)) rpm[j]=*v;
      return f;
    }
    return {};
  }
 private: const Options &options_;
};
class Trial {
 public:
  explicit Trial(Options options):options_(std::move(options)) {
    BusConfig config;config.axes=options_.axes;
    config.program_ack_timeout=std::chrono::milliseconds(options_.ack_ms);
    config.program_position_tolerance.fill(options_.tolerance_deg*pi/180);
    // Odstępstwo stanowiska 1 RPM od domyślnego holding=3 RPM. Bez nowej zgody
    // ten program nie zwiększa prędkości ani acc ponad wcześniejszy zakres.
    for(unsigned j=0;j<6;++j) {
      config.axes[j].streaming.min_holding_rpm=1;config.axes[j].streaming.max_rpm=1;
      config.axes[j].streaming.acceleration=config.axes[j].acceleration_code;
      config.program_streaming[j]=config.axes[j].streaming;
    }
    auto transport=std::make_unique<GuardedTransport>(options_);bus=transport.get();
    manager=std::make_unique<CanBusManager>(config,std::move(transport),Clock::now);
    manager->require_configuration();manager->open(Clock::now());
    if(!manager->control_state().fault && !bus->is_open()) throw std::runtime_error("Nie otwarto CAN");
    RobotProfile profile;profile.revision="ptp-group-bench";
    for(unsigned j=0;j<6;++j) if(options_.selected[j]) {
      const auto &a=options_.axes[j];profile.changed[j]=true;
      profile.axes[j]={true,a.gear_ratio,a.direction,a.offset,a.lower,a.upper,a.max_velocity,a.max_acceleration};
    }
    if(!manager->configure(ConfigurationPhase::bootstrap,"group-boot","",profile,Clock::now()).accepted ||
       !manager->configure(ConfigurationPhase::finish,"group-boot","",profile,Clock::now()).accepted)
      throw std::runtime_error("Profil próby odrzucony");
  }
  void tick(bool allow_fault=false) {
    if(interrupted && !allow_fault) throw std::runtime_error("Przerwanie operatora");
    const auto now=Clock::now();if(previous!=Time{}) max_gap=std::max(max_gap,std::chrono::duration<double,std::milli>(now-previous).count());
    previous=now;manager->tick(now);
    if(manager->control_state().fault && !allow_fault) throw std::runtime_error(manager->snapshot().message);
    const auto after=Clock::now();due+=std::chrono::microseconds(options_.tick_us);if(due<after) due=after;
    std::this_thread::sleep_until(due);
  }
  template<class Predicate> void wait(Predicate predicate,double timeout,const std::string &message) {
    const auto deadline=Clock::now()+std::chrono::duration<double>(timeout);
    while(!predicate()) { if(Clock::now()>=deadline) throw std::runtime_error(message);tick(); }
  }
  void run() {
    wait([&]{return !manager->snapshot().scanning;},5,"Timeout skanu");
    for(unsigned j=0;j<6;++j) if(options_.selected[j]) {
      wait([&]{return manager->manual().snapshot(Clock::now()).axes[j].available;},2,"Brak pomiaru przed ENABLE");
      if(!manager->manual().set_enable(j+1,true,Clock::now())) throw std::runtime_error("Odrzucono ENABLE J"+std::to_string(j+1));
      wait([&]{return !manager->manual().busy();},3,"Timeout ENABLE");
      if(!manager->manual().prepared(j+1)) throw std::runtime_error(manager->manual().snapshot(Clock::now()).message);
      bus->microstep[j]=manager->manual().microstep(j+1);
      if(bus->microstep[j]!=16) throw std::runtime_error("Próba wymaga potwierdzonego mikrokroku 16");
    }
    // ENABLE może ustawić rotor przed pierwszym punktem. Pozycja początkowa
    // programu musi pochodzić z postoju, a nie z przejściowego odczytu.
    const auto enabled_at=Clock::now();
    auto anchor=bus->angle;auto stable_since=Clock::now();
    wait([&] {
      const auto now=Clock::now();bool fresh=true;
      for(unsigned j=0;j<6;++j) if(options_.selected[j]) {
        fresh=fresh && bus->stamp[j]!=Time{} && now-bus->stamp[j]<60ms;
        if(std::abs(bus->angle[j]-anchor[j])>3*360./16384.) {
          anchor=bus->angle;stable_since=now;
        }
      }
      return fresh && now-enabled_at>=1s && now-stable_since>=200ms;
    },5,"Brak stabilnego postoju po ENABLE");
    for(unsigned j=0;j<6;++j) if(options_.selected[j] && std::abs(bus->angle[j])>.5)
      throw std::runtime_error("Po ENABLE pozycja poza zerem J"+std::to_string(j+1));
    note("start_stable","Postój po ENABLE: >=1 s, zmiana <=3 impulsy przez >=200 ms");
    auto authorization=manager->config();authorization.program_enabled=true;authorization.program_revision="ptp-group-bench";
    manager->configure_program_authorization(authorization);manager->update_motion_model("measured-joint-profile");
    leg(20,"plus20");leg(0,"return0");
  }
  void leg(double target,const std::string &label) {
    PreparedMotionProgram p;p.id=label;p.revision="ptp-group-bench";p.model_id="measured-joint-profile";p.settings_revision="bench-settings";
    manager->update_motion_settings(p.settings_revision,true,Clock::now());
    const auto state=manager->control_state();double minimum_duration=0;
    for(unsigned j=0;j<6;++j) if(options_.selected[j])
      minimum_duration=std::max(minimum_duration,1.92*std::abs(target*pi/180-state.positions[j].value)/options_.axes[j].max_velocity);
    const double duration=std::max(.2,std::ceil(options_.duration_scale*minimum_duration/kMotionCycleSeconds)*kMotionCycleSeconds);
    const unsigned count=std::lround(duration/kMotionCycleSeconds);
    std::cout<<"{\"event\":\"trajectory\",\"label\":"<<std::quoted(label)
      <<",\"duration\":"<<std::setprecision(17)<<duration<<",\"target_deg\":"<<target
      <<",\"start_deg\":[";
    for(unsigned j=0;j<6;++j) {if(j) std::cout<<',';std::cout<<state.positions[j].value*180/pi;}
    std::cout<<"],\"axis_profiles\":[";
    for(unsigned j=0;j<6;++j) {
      if(j) std::cout<<',';
      const auto &a=options_.axes[j];
      std::cout<<"{\"gear_ratio\":"<<a.gear_ratio<<",\"direction\":"<<a.direction<<",\"offset_rad\":"<<a.offset<<'}';
    }
    std::cout<<"],\"points\":"<<(count+1)<<",\"sample_ms\":10,\"use_ack\":false,\"ack_ms\":"<<options_.ack_ms
      <<",\"tolerance_deg\":"<<options_.tolerance_deg<<"}"<<std::endl;
    for(unsigned k=0;k<=count;++k) {
      const double t=k*kMotionCycleSeconds,u=t/duration;ProgramPoint q;q.seconds=t;
      for(unsigned j=0;j<6;++j) if(options_.selected[j]) {
        const double start=state.positions[j].value,d=target*pi/180-start;
        q.positions[j]=start+d*(10*std::pow(u,3)-15*std::pow(u,4)+6*std::pow(u,5));
        q.velocities[j]=d*(30*u*u-60*std::pow(u,3)+30*std::pow(u,4))/duration;
        q.accelerations[j]=d*(60*u-180*u*u+120*std::pow(u,3))/(duration*duration);
      }
      p.points.push_back(q);
    }
    const auto prepared=manager->prepare_program(std::move(p),Clock::now());
    note("prepared",label+" "+prepared.mode+" duration="+std::to_string(duration));
    if(!manager->execute_program(prepared.program_id,label,Clock::now())) throw std::runtime_error("Odrzucono start "+label);
    auto began=Clock::now();std::string last;
    while(manager->program_status().busy) {
      if(Clock::now()-began>30s) throw std::runtime_error("Zewnętrzny limit próby 30 s: "+manager->program_status().state+" "+manager->program_status().message);
      tick();const auto status=manager->program_status();
      if(status.state!=last && status.state!="staging" && status.state!="running") {note("state",label+" "+status.state+": "+status.message);last=status.state;}
    }
    const auto status=manager->program_status();
    if(status.state!="succeeded") throw std::runtime_error(status.message);
    for(unsigned j=0;j<6;++j) if(options_.selected[j] && std::abs(bus->angle[j]-target)>options_.tolerance_deg)
      throw std::runtime_error("Końcowa pozycja poza tolerancją J"+std::to_string(j+1));
    note("leg_pass",label+" seconds="+std::to_string(std::chrono::duration<double>(Clock::now()-began).count()));
  }
  void summary(bool passed,const std::string &message) {
    const auto s=manager->snapshot().statistics;
    std::cout<<"{\"event\":\"trial_result\",\"success\":"<<(passed?"true":"false")<<",\"message\":"<<std::quoted(message)
      <<",\"axes\":"<<std::quoted(options_.axis_list)<<",\"sync_count\":"<<bus->syncs<<",\"max_io_gap_ms\":"<<max_gap
      <<",\"ack_margin_min_ms\":"<<s.ack_margin_ms.minimum<<",\"rtt_max_ms\":"<<s.rtt.maximum<<",\"timeouts\":"<<s.timeouts
      <<",\"peak_utilization\":"<<s.peak_scheduled_utilization<<",\"positions\":[";
    for(unsigned j=0;j<6;++j) {if(j) std::cout<<',';std::cout<<bus->angle[j];}
    std::cout<<"],\"max_positions\":[";
    for(unsigned j=0;j<6;++j) {if(j) std::cout<<',';std::cout<<bus->peak[j];}
    std::cout<<"]}"<<std::endl;
  }
  void cleanup() {
    manager->stop("Koniec próby sprzętowej");const auto deadline=Clock::now()+60ms;
    while(manager->snapshot().statistics.stop_pending && Clock::now()<deadline) tick(true);
    for(unsigned j=0;j<6;++j) if(options_.selected[j]) bus->socket.send(j+1,enable_request(j+1,false));
    manager->close();
  }
 private:
  Options options_;
  std::unique_ptr<CanBusManager> manager;
  GuardedTransport *bus{};
  Time due=Clock::now(),previous{};
  double max_gap{};
};
}
int main(int argc,char **argv) {
  std::signal(SIGINT,signal_handler);std::signal(SIGTERM,signal_handler);
  try {
    auto opts=options(argc,argv);note("trial_begin",opts.axis_list);
    Trial trial(std::move(opts));bool passed=false;std::string message;
    try {trial.run();passed=true;message="+20° i powrót do 0° w tolerancji";}
    catch(const std::exception &e) {message=e.what();note("failure",message);}
    trial.summary(passed,message);
    try {interrupted=0;trial.cleanup();note("cleanup_sent");}
    catch(const std::exception &e) {note("cleanup_error",e.what());passed=false;}
    return passed?0:1;
  } catch(const std::exception &e) {note("startup_error",e.what());return 2;}
}
