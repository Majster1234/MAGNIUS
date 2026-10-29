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
namespace {
double polynomial(const std::vector<double> &c,double u) {
  double value=0;for(auto i=c.rbegin();i!=c.rend();++i) value=value*u+*i;return value;
}
// Izolacja pierwiastków przez miejsca zerowe pochodnej. Obliczana przed ruchem.
std::vector<double> unit_roots(std::vector<double> c) {
  double scale=0;for(double x:c) scale=std::max(scale,std::abs(x));
  const double epsilon=std::max(1e-14,scale*1e-12);
  while(c.size()>1 && std::abs(c.back())<epsilon) c.pop_back();
  if(c.size()==1) return {};
  std::vector<double> derivative;for(unsigned i=1;i<c.size();++i) derivative.push_back(i*c[i]);
  auto splits=unit_roots(derivative);splits.insert(splits.begin(),0.);splits.push_back(1.);
  std::vector<double> roots;
  for(double u:splits) if(std::abs(polynomial(c,u))<=epsilon) roots.push_back(u);
  for(unsigned i=1;i<splits.size();++i) {
    double lo=splits[i-1],hi=splits[i],left=polynomial(c,lo),right=polynomial(c,hi);
    if(left*right>=0) continue;
    for(unsigned n=0;n<50;++n) {
      const double mid=(lo+hi)/2,value=polynomial(c,mid);
      if(left*value<=0) hi=mid;else {lo=mid;left=value;}
    }
    roots.push_back((lo+hi)/2);
  }
  std::sort(roots.begin(),roots.end());
  roots.erase(std::unique(roots.begin(),roots.end(),[](double a,double b){return std::abs(a-b)<1e-10;}),roots.end());
  return roots;
}
}
void compile_program_reversals(PreparedMotionProgram &p) {
  p.reversal_times.clear();
  for(unsigned k=1;k<p.points.size();++k) {
    const auto &a=p.points[k-1],&b=p.points[k];const double h=b.seconds-a.seconds;
    for(unsigned j=0;j<6;++j) if(p.participating[j]) {
      const double c1=a.velocities[j]*h,c2=a.accelerations[j]*h*h/2;
      const double d=b.positions[j]-a.positions[j],v=b.velocities[j]*h,acc=b.accelerations[j]*h*h;
      const double c3=10*d-6*c1-3*c2-4*v+acc/2,c4=-15*d+8*c1+3*c2+7*v-acc,c5=6*d-3*c1-c2-3*v+acc/2;
      for(double u:unit_roots({c1,2*c2,3*c3,4*c4,5*c5})) {
        const double t=a.seconds+u*h,delta=std::min(1e-6,h*1e-4);
        if(t<=0 || t>=p.duration) continue;
        const double before=program_reference(p,std::max(0.,t-delta)).velocities[j];
        const double after=program_reference(p,std::min(p.duration,t+delta)).velocities[j];
        if(before*after<0 || ((std::abs(before)>1e-14)!=(std::abs(after)>1e-14)))
          p.reversal_times.push_back(t);
      }
    }
  }
  std::sort(p.reversal_times.begin(),p.reversal_times.end());
  p.reversal_times.erase(std::unique(p.reversal_times.begin(),p.reversal_times.end(),
      [](double a,double b){return std::abs(a-b)<1e-9;}),p.reversal_times.end());
}
double program_lookahead_time(const PreparedMotionProgram &p,double phase,double horizon) {
  const auto next=std::upper_bound(p.reversal_times.begin(),p.reversal_times.end(),phase+1e-9);
  return std::min({p.duration,phase+horizon,next==p.reversal_times.end() ? p.duration : *next});
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
