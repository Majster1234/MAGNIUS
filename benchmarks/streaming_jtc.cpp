#include "harness.hpp"
#include "streaming_accuracy.hpp"
#include "../ros2_ws/src/mks_servo_can/test/streaming_fixture.hpp"
#include <cmath>
#include <cstdlib>
#include <thread>
using namespace streaming_test;
namespace {
BusConfig profile() {auto c=config();c.motion_enabled=true;return c;}
struct Cycle {
  Rig h{profile()};std::uint64_t count{};
  Cycle() {
    h.ready();
    for(unsigned j=1;j<=6;++j) {
      if(!h.bus->manual().set_enable(j,true,h.now)) throw std::runtime_error("ENABLE");
      h.run(200);
    }
    if(!h.bus->activate(Source::trajectory,h.now)) throw std::runtime_error("JTC activation");
  }
  void cycle(std::uint64_t) {
    CommandBatch batch;const double t=++count*.01;
    batch.positions.fill(.05*std::sin(t));batch.velocities.fill(.05*std::cos(t));
    batch.velocities_valid=true;batch.created=h.now;batch.expires=h.now+20ms;batch.sequence=count;batch.source=Source::trajectory;
    h.port->events.clear();
    tag_generation(batch,*h.bus,0);
    if(!h.bus->submit(batch)) throw std::runtime_error("JTC submit");
    h.run(10);
  }
  bool verify() const {return !h.bus->snapshot().fault && h.port->counts[0xf5]>=6*count && !h.port->counts[0x4b];}
};
}
int main(int argc,char **argv) {
  if(argc==2 && std::string(argv[1])=="--accuracy") return streaming_accuracy::report();
  if(const auto duration=std::getenv("CAN_BENCH_PTP_REALTIME_SECONDS")) {
    Cycle cycle;const auto start=Clock::now();auto due=start;
    while(std::chrono::duration<double>(Clock::now()-start).count()<std::stod(duration)) {
      cycle.cycle(0);due+=10ms;std::this_thread::sleep_until(due);
    }
    return cycle.verify() ? 0 : 1;
  }
  return robot_benchmarks::main(argc,argv,[]{return Cycle{};});
}
