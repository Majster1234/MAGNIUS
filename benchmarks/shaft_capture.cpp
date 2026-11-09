// Ten sam emulator i cykle przed/po; bez fizycznego CAN.
#define main schedule_benchmark_main
#include "can_schedule.cpp"
#undef main
#include <ctime>
#include <iomanip>
int main() {
  Harness h;h.ready();
  auto cycle=[&] {
    h.run(20);
#ifdef MKS_SHAFT_TRACE_AVAILABLE
    std::array<ShaftPositionSample,256> samples;
    h.manager->drain_shaft_trace(samples.data(),samples.size());
#endif
  };
  for(unsigned i=0;i<1000;++i) cycle();
  std::cout<<std::setprecision(12)<<"{\"iterations\":10000,\"samples\":[";
  for(unsigned series=0;series<7;++series) {
    const auto before=Clock::now();const auto cpu=std::clock();
    for(unsigned i=0;i<10000;++i) cycle();
    if(series) std::cout<<',';
    std::cout<<"{\"wall_us\":"<<std::chrono::duration<double,std::micro>(Clock::now()-before).count()/10000
      <<",\"cpu_us\":"<<double(std::clock()-cpu)/CLOCKS_PER_SEC*1e6/10000<<'}';
  }
  const auto s=h.manager->snapshot();
  std::cout<<"],\"tx\":"<<s.statistics.tx<<",\"rx\":"<<s.statistics.rx<<",\"position_requests\":[";
  for(unsigned i=1;i<=6;++i) {if(i>1) std::cout<<',';std::cout<<h.bus->position_requests[i];}
  std::cout<<"]}"<<std::endl;
}
