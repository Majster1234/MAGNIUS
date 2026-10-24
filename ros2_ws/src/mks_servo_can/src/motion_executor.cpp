#include "mks_servo_can/motion_executor.hpp"
#include "mks_servo_can/bus_manager.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mks_servo_can {
ProgramPoint program_reference(const PreparedMotionProgram &p,double t) {
  if(p.points.empty()) throw std::invalid_argument("Pusty program");
  if(t<=0) return p.points.front();
  if(t>=p.duration) return p.points.back();
  auto right=std::upper_bound(p.points.begin(),p.points.end(),t,[](double value,const auto &point){return value<point.seconds;});
  const auto &a=*(right-1),&b=*right;
  const double dt=b.seconds-a.seconds,u=(t-a.seconds)/dt;
  ProgramPoint out;out.seconds=t;
  for(unsigned j=0;j<6;++j) {
    const double c0=a.positions[j],c1=a.velocities[j]*dt,c2=a.accelerations[j]*dt*dt/2;
    const double d=b.positions[j]-c0,v=b.velocities[j]*dt,acc=b.accelerations[j]*dt*dt;
    const double c3=10*d-6*c1-3*c2-4*v+acc/2,c4=-15*d+8*c1+3*c2+7*v-acc,c5=6*d-3*c1-c2-3*v+acc/2;
    out.positions[j]=c0+u*(c1+u*(c2+u*(c3+u*(c4+u*c5))));
    out.velocities[j]=(c1+u*(2*c2+u*(3*c3+u*(4*c4+u*5*c5))))/dt;
    out.accelerations[j]=(2*c2+u*(6*c3+u*(12*c4+u*20*c5)))/(dt*dt);
  }
  return out;
}
std::array<double,2> program_position_bounds(const PreparedMotionProgram &p,unsigned axis,double begin,double end) {
  begin=std::clamp(begin,0.,p.duration);end=std::clamp(end,begin,p.duration);
  auto left=program_reference(p,begin);std::array<double,2> bounds{left.positions[axis],left.positions[axis]};
  while(begin<end) {
    const auto next=std::upper_bound(p.points.begin(),p.points.end(),begin,[](double t,const auto &point){return t<point.seconds;});
    const auto finish=next==p.points.end() ? end : std::min(end,next->seconds);
    const auto right=program_reference(p,finish);const auto h=finish-begin;
    const auto q=left.positions[axis],v=left.velocities[axis],a=left.accelerations[axis];
    const auto r=right.positions[axis],w=right.velocities[axis],b=right.accelerations[axis];
    const std::array<double,6> hull{q,q+v*h/5,q+2*v*h/5+a*h*h/20,r-2*w*h/5+b*h*h/20,r-w*h/5,r};
    for(double value:hull) { bounds[0]=std::min(bounds[0],value);bounds[1]=std::max(bounds[1],value); }
    begin=finish;left=right;
  }
  return bounds;
}
PreparedMotionProgram compile_motion_program(PreparedMotionProgram p,const std::array<AxisConfig,6> &axes,
    const std::array<std::uint16_t,6> &microsteps,double dt) {
  if(!std::isfinite(dt) || std::abs(dt-kStreamingCycleSeconds)>1e-12)
    throw std::invalid_argument("Streaming wymaga cyklu 10 ms");
  return compile_streaming_program(std::move(p),axes,microsteps);
}
}
