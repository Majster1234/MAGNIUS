#include "mks_servo_can/streaming_motion.hpp"
#include "mks_servo_can/bus_manager.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mks_servo_can {
namespace { constexpr double pi=3.14159265358979323846; }
StreamingSetpoint streaming_setpoint(const AxisConfig &a,double q,double v,std::uint16_t microstep) {
  StreamingState state;
  return streaming_setpoint(a,q,v,state,std::nullopt,microstep);
}
StreamingSetpoint streaming_setpoint(const AxisConfig &a,double q,double v,StreamingState &state,
    std::optional<double> actual,std::uint16_t microstep) {
  const auto &s=a.streaming;
  if(!std::isfinite(q) || !std::isfinite(v) || !std::isfinite(a.gear_ratio) || a.gear_ratio<=0 ||
      (a.direction!=1 && a.direction!=-1) || !std::isfinite(a.offset) ||
      !std::isfinite(a.lower) || !std::isfinite(a.upper) || a.lower>=a.upper || q<a.lower-1e-9 || q>a.upper+1e-9 ||
      !std::isfinite(a.max_velocity) || a.max_velocity<=0 || std::abs(v)>a.max_velocity+1e-6 ||
      !std::isfinite(s.lookahead_time_sec) || s.lookahead_time_sec<0 || s.lookahead_time_sec>.2 ||
      !std::isfinite(s.velocity_ff_scale) || s.velocity_ff_scale<1 || s.velocity_ff_scale>3 ||
      !std::isfinite(s.position_kp) || s.position_kp<0 ||
      (actual && !std::isfinite(*actual)) || !std::isfinite(state.rpm_dither_accum) ||
      state.rpm_dither_accum<0 || state.rpm_dither_accum>=1 ||
      s.encoder_cpr!=16384 || !s.min_holding_rpm || s.max_rpm>3000 || s.max_rpm<s.min_holding_rpm ||
      a.address<1 || a.address>2047)
    throw std::invalid_argument("Nieprawidłowy profil/próbka streaming F5");
  const double scale=speed_scale(microstep),joint_to_rpm=a.gear_ratio*60/(2*pi);
  const auto maximum=std::min(3000L,static_cast<long>(std::floor(std::min<double>(s.max_rpm,a.max_velocity*joint_to_rpm)/scale+1e-9)));
  const auto minimum=static_cast<long>(std::ceil(s.min_holding_rpm/scale-1e-9));
  if(maximum<minimum || maximum>3000) throw std::invalid_argument("min_holding_rpm przekracza limit osi lub kwantyzację MKS");
  StreamingSetpoint out;
  out.lookahead_position=std::clamp(q+v*s.lookahead_time_sec,a.lower,a.upper);
  const double pulses=a.direction*(out.lookahead_position-a.offset)*a.gear_ratio*s.encoder_cpr/(2*pi);
  if(!std::isfinite(pulses)) throw std::invalid_argument("Nieprzedstawialne impulsy streaming");
  out.ticks=static_cast<std::int32_t>(std::llround(std::clamp(pulses,-8388608.,8388607.)));
  const int direction=(v>0)-(v<0);
  if(direction!=state.direction) {state.rpm_dither_accum=0;state.direction=direction;}
  const double correction=actual && direction ? s.position_kp*(q-*actual)*direction : 0.;
  const double effective=std::clamp(std::abs(v)*s.velocity_ff_scale+correction,0.,a.max_velocity);
  const double rpm=std::min(effective*joint_to_rpm/scale,static_cast<double>(maximum));
  if(rpm<minimum) {out.rpm=minimum;state.rpm_dither_accum=0;}
  else {
    const double with_residual=rpm+state.rpm_dither_accum;
    const auto integral=static_cast<long>(std::floor(with_residual+1e-12));
    state.rpm_dither_accum=std::max(0.,with_residual-integral);
    out.rpm=static_cast<std::uint16_t>(std::clamp(integral,minimum,maximum));
  }
  const auto bits=static_cast<std::uint32_t>(out.ticks);
  out.bytes={0xf5,static_cast<std::uint8_t>((out.rpm>>8)&0x0f),static_cast<std::uint8_t>(out.rpm),
      s.acceleration,static_cast<std::uint8_t>(bits>>16),static_cast<std::uint8_t>(bits>>8),static_cast<std::uint8_t>(bits),0};
  unsigned sum=a.address;for(unsigned i=0;i<7;++i) sum+=out.bytes[i];out.bytes[7]=static_cast<std::uint8_t>(sum);
  return out;
}
double streaming_bus_utilization(unsigned axes,unsigned bitrate) {
  if(axes>6 || (bitrate!=500000 && bitrate!=1000000)) throw std::invalid_argument("Bitrate streaming: 500 kbit/s lub 1 Mbit/s");
  const double motion=100*(can_frame_bits(8)+can_frame_bits(2)+can_frame_bits(8));
  const double diagnostics=20*(can_frame_bits(2)+can_frame_bits(3))+
      20*(can_frame_bits(2)+can_frame_bits(4))+20*(can_frame_bits(2)+can_frame_bits(3));
  return axes*(motion+diagnostics)/bitrate;
}
PreparedMotionProgram compile_streaming_program(PreparedMotionProgram p,const std::array<AxisConfig,6> &axes,
    const std::array<std::uint16_t,6> &microsteps) {
  if(p.points.size()<2 || p.points.size()>10000 || p.points.front().seconds!=0)
    throw std::invalid_argument("Streaming wymaga 2–10000 punktów od t=0");
  p.duration=p.points.back().seconds;
  if(!std::isfinite(p.duration) || p.duration<=0 || p.duration>200) throw std::invalid_argument("Nieprawidłowy czas programu");
  double previous=-1;bool any=false;p.tcp_verified=true;
  for(unsigned j=0;j<6;++j) {
    p.start[j]=p.points.front().positions[j];p.target[j]=p.points.back().positions[j];
    p.tcp_verified=p.tcp_verified && p.participating[j];any=any || p.participating[j];
    if(p.participating[j] && (std::abs(p.points.front().velocities[j])>1e-8 || std::abs(p.points.back().velocities[j])>1e-8))
      throw std::invalid_argument("PTP wymaga prędkości 0 na obu końcach");
  }
  if(!any) throw std::invalid_argument("Brak uczestników streaming");
  for(const auto &point:p.points) {
    if(!std::isfinite(point.seconds) || point.seconds<=previous) throw std::invalid_argument("Nieciągły czas trajektorii");
    previous=point.seconds;
    for(unsigned j=0;j<6;++j) if(!std::isfinite(point.positions[j]) || !std::isfinite(point.velocities[j]) || !std::isfinite(point.accelerations[j]))
      throw std::invalid_argument("Nieskończona próbka trajektorii");
  }
  // Otoczki Bernsteina: limity całych przedziałów, a nie samych węzłów 10 ms.
  for(std::size_t k=1;k<p.points.size();++k) {
    const auto &l=p.points[k-1],&r=p.points[k];const double h=r.seconds-l.seconds;
    for(unsigned j=0;j<6;++j) if(p.participating[j]) {
      const auto &a=axes[j];
      if(!std::isfinite(a.max_acceleration) || a.max_acceleration<=0) throw std::invalid_argument("Brak limitu przyspieszenia");
      const std::array<double,6> b{l.positions[j],l.positions[j]+l.velocities[j]*h/5,
        l.positions[j]+2*l.velocities[j]*h/5+l.accelerations[j]*h*h/20,
        r.positions[j]-2*r.velocities[j]*h/5+r.accelerations[j]*h*h/20,r.positions[j]-r.velocities[j]*h/5,r.positions[j]};
      for(unsigned i=0;i<6;++i) if(!std::isfinite(b[i]) || b[i]<a.lower-1e-9 || b[i]>a.upper+1e-9 ||
          (i<5 && std::abs(5*(b[i+1]-b[i])/h)>a.max_velocity+1e-6) ||
          (i<4 && std::abs(20*(b[i+2]-2*b[i+1]+b[i])/(h*h))>a.max_acceleration+1e-5))
        throw std::invalid_argument("Trajektoria streaming przekracza limity osi");
    }
  }
  const auto count=static_cast<std::size_t>(std::ceil(p.duration/kStreamingCycleSeconds));
  p.mode="streamed";p.frames.clear();p.frames.reserve(count+1);
  std::array<StreamingState,6> states{};
  for(std::size_t k=0;k<=count;++k) {
    ProgramFrame frame;frame.seconds=k*kStreamingCycleSeconds;
    const auto q=program_reference(p,std::min(p.duration,frame.seconds));
    for(unsigned j=0;j<6;++j) if(p.participating[j]) {
      const auto sample=streaming_setpoint(axes[j],q.positions[j],q.velocities[j],states[j],std::nullopt,microsteps[j]);
      frame.payloads[j].assign(sample.bytes.begin(),sample.bytes.end());
    }
    p.frames.push_back(std::move(frame));
  }
  return p;
}
}
