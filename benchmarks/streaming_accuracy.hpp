#pragma once
#include "mks_servo_can/bus_manager.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

// Model syntetyczny: pozycjoner ograniczony RPM, enkoder 16384/obrót,
// idealny krok 1 ms, opóźnienie pomiaru 11,2 ms. Nie modeluje dynamiki robota.
namespace streaming_accuracy {
using namespace mks_servo_can;
constexpr double pi=3.14159265358979323846,dt=.001,leg_time=2.,pause=.5,delay=.0112;
struct Reference {double q,v,a,remaining;};
inline Reference reference(double t) {
  bool back=t>=leg_time+pause;
  const double elapsed=back ? t-leg_time-pause : t;
  const double u=std::clamp(elapsed/leg_time,0.,1.),amplitude=(back ? -20. : 20.)*pi/180;
  return {(back ? 20.*pi/180 : 0.)+amplitude*(10*std::pow(u,3)-15*std::pow(u,4)+6*std::pow(u,5)),
    amplitude*(30*u*u-60*u*u*u+30*std::pow(u,4))/leg_time,
    amplitude*(60*u-180*u*u+120*u*u*u)/(leg_time*leg_time),std::max(0.,leg_time-elapsed)};
}
// Zamrożony wariant sprzed poprawki. Nie wywołuje zmienianych wrapperów.
inline StreamingSetpoint legacy(const AxisConfig &a,double q,double v,StreamingState &state,double actual) {
  const auto &s=a.streaming;StreamingSetpoint out;
  out.lookahead_position=std::clamp(q+v*s.lookahead_time_sec,a.lower,a.upper);
  out.ticks=std::llround(a.direction*(out.lookahead_position-a.offset)*a.gear_ratio*16384/(2*pi));
  const int direction=(v>0)-(v<0);
  if(direction!=state.direction) {state.rpm_dither_accum=0;state.direction=direction;}
  const double correction=direction ? s.position_kp*(q-actual)*direction : 0;
  const double rpm=std::min(std::clamp(std::abs(v)*s.velocity_ff_scale+correction,0.,a.max_velocity)*a.gear_ratio*60/(2*pi),double(s.max_rpm));
  if(rpm<s.min_holding_rpm) {out.rpm=s.min_holding_rpm;state.rpm_dither_accum=0;}
  else {const double value=rpm+state.rpm_dither_accum;out.rpm=std::floor(value+1e-12);state.rpm_dither_accum=std::max(0.,value-out.rpm);}
  return out;
}
// Ten sam adapter kompiluje się przed rozszerzeniem API i po nim.
template<class A> auto patched(const A &a,const Reference &r,StreamingState &s,double actual,double age,int)
  -> decltype(streaming_setpoint(a,r.q,r.v,s,actual,16,r.a,r.remaining,age)) {
  return streaming_setpoint(a,r.q,r.v,s,actual,16,r.a,r.remaining,age);
}
template<class A> StreamingSetpoint patched(const A &a,const Reference &r,StreamingState &s,double actual,double,long) {
  return streaming_setpoint(a,r.q,r.v,s,actual,16);
}
struct Result {double sum2{},maximum{};unsigned samples{};std::array<double,2> settling{{-1,-1}};};
inline Result simulate(unsigned axes,bool corrected) {
  std::vector<AxisConfig> config(axes);std::vector<StreamingState> states(axes);
  std::vector<double> positions(axes),targets(axes),speeds(axes);
  std::vector<std::vector<double>> history(axes,std::vector<double>{0});Result result;
  std::array<double,2> entered{{-1,-1}};
  for(unsigned j=0;j<axes;++j) config[j]={static_cast<uint16_t>(j+1),double(j+3),j%2 ? -1. : 1.,0,-1,1,1,2,.2,135};
  const unsigned steps=std::llround((2*leg_time+2*pause)/dt);
  for(unsigned k=0;k<=steps;++k) {
    const double t=k*dt;const auto r=reference(t);
    const bool sending=t<=leg_time+1e-9 || (t>=leg_time+pause-1e-9 && t<=2*leg_time+pause+1e-9);
    if(k%10==0 && sending) for(unsigned j=0;j<axes;++j) {
      const double when=std::max(0.,t-delay),index=when/dt;
      const auto left=std::min(history[j].size()-1,static_cast<std::size_t>(index)),right=std::min(history[j].size()-1,left+1);
      const double measured=history[j][left]+(history[j][right]-history[j][left])*(index-left);
      const auto &a=config[j];const double pulses=std::llround(measured*a.direction*a.gear_ratio*16384/(2*pi));
      const double actual=pulses*2*pi/(16384*a.direction*a.gear_ratio);
      const auto command=corrected ? patched(a,r,states[j],actual,t-when,0) : legacy(a,r.q,r.v,states[j],actual);
      targets[j]=command.ticks*2*pi/(16384*a.direction*a.gear_ratio);
      speeds[j]=command.rpm*2*pi/(60*a.gear_ratio);
    }
    bool inside=true;
    for(unsigned j=0;j<axes;++j) {
      const double error=(positions[j]-r.q)*180/pi;
      result.sum2+=error*error;result.maximum=std::max(result.maximum,std::abs(error));++result.samples;
      inside=inside && std::abs(error)<=.5;
    }
    const unsigned leg=t<leg_time+pause ? 0 : 1;
    const double finish=leg==0 ? leg_time : 2*leg_time+pause;
    if(t>=finish) {
      if(!inside) entered[leg]=-1;
      else if(entered[leg]<0) entered[leg]=t;
      if(entered[leg]>=0 && t-entered[leg]>=.1-1e-9 && result.settling[leg]<0)
        result.settling[leg]=1000*(entered[leg]-finish);
    }
    for(unsigned j=0;j<axes;++j) {
      positions[j]+=std::clamp(targets[j]-positions[j],-speeds[j]*dt,speeds[j]*dt);
      history[j].push_back(positions[j]);
    }
  }
  return result;
}
inline int report() {
  std::cout<<"{\"model\":\"ideal RPM-limited position follower; synthetic ratios 3..8; no firmware dynamics\",\"feedback_delay_ms\":11.2,\"cycle_ms\":10,\"leg_duration_sec\":2,\"settling_observation_sec\":0.5,\"results\":[";
  bool first=true;
  for(unsigned axes:{4u,6u}) for(bool corrected:{false,true}) {
    const auto r=simulate(axes,corrected);if(!first) std::cout<<',';first=false;
    std::cout<<"{\"axes\":"<<axes<<",\"variant\":\""<<(corrected ? "patched" : "baseline")<<"\",\"rmse_deg\":"<<std::sqrt(r.sum2/r.samples)<<",\"max_error_deg\":"<<r.maximum<<",\"settling_time_ms\":";
    if(r.settling[0]<0 || r.settling[1]<0) std::cout<<"null";else std::cout<<std::max(r.settling[0],r.settling[1]);
    std::cout<<",\"leg_settling_time_ms\":["<<r.settling[0]<<','<<r.settling[1]<<"]}";
  }
  std::cout<<"]}\n";return 0;
}
}
