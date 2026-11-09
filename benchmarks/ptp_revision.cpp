#include "harness.hpp"
#include "../ros2_ws/src/mks_servo_can/test/streaming_fixture.hpp"
using namespace streaming_test;
namespace {
struct Cycle {
  std::uint64_t cycles{}, accepted{}, rejected{};
  void cycle(std::uint64_t) {
    Rig h;h.ready();
    h.bus->prepare_program(h.program(),h.now);
    h.port->enabled.fill(false);h.run(350);
    RobotProfile profile;profile.revision="changed";profile.changed.fill(true);
    for(unsigned j=0;j<6;++j) {
      const auto &a=h.bus->config().axes[j];
      profile.axes[j]={true,1.+j,a.direction,a.offset,a.lower,a.upper,a.max_velocity,a.max_acceleration};
    }
    for(const auto phase:{ConfigurationPhase::prepare,ConfigurationPhase::commit,ConfigurationPhase::finish})
      if(!h.bus->configure(phase,"change","bench",profile,h.now).accepted) throw std::runtime_error("configuration");
    h.port->enabled.fill(true);h.run(100);
    auto p=h.program();p.revision=profile.revision;
    try {h.bus->prepare_program(p,h.now);++accepted;} catch(const std::runtime_error &) {++rejected;}
    if(h.bus->configure(ConfigurationPhase::status,"","",profile,h.now).revision!=profile.revision || h.port->counts[0xf5])
      throw std::runtime_error("revision/IO");
    ++cycles;
  }
  bool verify() const {return cycles && accepted+rejected==cycles && (!accepted || !rejected);}
};
}
int main(int argc,char **argv) {return robot_benchmarks::main(argc,argv,[]{return Cycle{};});}
