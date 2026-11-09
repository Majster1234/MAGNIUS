// Wyłącznie emulator streaming_fixture.hpp; ten program nie otwiera CAN ani ROS.
#include "streaming_fixture.hpp"
#include <chrono>
#include <iostream>
int main(int argc,char **) {
  double elapsed_ms=0;unsigned long f5=0,sync=0;
  for(unsigned i=0;i<100;++i) {
    streaming_test::Rig h;h.ready();
#ifdef SYNCHRONIZED_BENCHMARK
    if(argc>1) h.synchronized();
#else
    (void)argc;
#endif
    auto p=h.program(.2,1);
    const auto begin=std::chrono::steady_clock::now();
    h.execute(std::move(p));h.run(1600);
    elapsed_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    if(h.bus->program_status().state!="succeeded") {std::cerr<<h.bus->program_status().message<<'\n';return 2;}
    f5+=h.port->counts[0xf5];sync+=h.port->counts[0x4b];
  }
  std::cout<<"runs=100 simulated_execution_ms=160000 F5="<<f5<<" SYNC="<<sync<<" execution_ms="<<elapsed_ms<<'\n';
}
