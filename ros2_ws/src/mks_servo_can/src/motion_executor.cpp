#include "mks_servo_can/motion_executor.hpp"
#include "mks_servo_can/bus_manager.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mks_servo_can {
namespace {
constexpr double pi=3.14159265358979323846;
struct NativeProfile { double distance{},speed{},acceleration{},ramp{},cruise{},finish{}; };
double native_position(const NativeProfile &p,double t) {
  t=std::clamp(t,0.,p.finish);
  if(t<=p.ramp) return .5*p.acceleration*t*t;
  if(t<=p.ramp+p.cruise) return .5*p.acceleration*p.ramp*p.ramp+p.speed*(t-p.ramp);
  return p.distance-.5*p.acceleration*(p.finish-t)*(p.finish-t);
}
}
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
  if(p.points.size()<2 || p.points.size()>10000 || !std::isfinite(dt) || std::abs(dt-kMotionCycleSeconds)>1e-12)
    throw std::invalid_argument("Program wymaga 2–10000 punktów i poprawnego cyklu");
  p.duration=p.points.back().seconds;
  if(p.points.front().seconds!=0 || !std::isfinite(p.duration) || p.duration<=0 || p.duration>200)
    throw std::invalid_argument("Program musi zaczynać się w t=0 i trwać do 200 s");
  bool any=false;p.tcp_verified=true;
  for(unsigned j=0;j<6;++j) if(p.participating[j]) {
    const auto &a=axes[j];
    if(a.address!=j+1 || !std::isfinite(a.gear_ratio) || a.gear_ratio<=0 || (a.direction!=1 && a.direction!=-1) ||
        !std::isfinite(a.offset) || !std::isfinite(a.lower) || !std::isfinite(a.upper) || a.lower>=a.upper ||
        !std::isfinite(a.max_velocity) || a.max_velocity<=0 || !std::isfinite(a.max_acceleration) || a.max_acceleration<=0)
      throw std::invalid_argument("Niepełny profil uczestniczącej osi");
  }
  for(unsigned j=0;j<6;++j) {
    any=any || p.participating[j];p.tcp_verified=p.tcp_verified && p.participating[j];
    p.start[j]=p.points.front().positions[j];p.target[j]=p.points.back().positions[j];
    if(p.participating[j] && (speed_scale(microsteps[j])<=0 || axes[j].tracking_error<=0 ||
        !std::isfinite(axes[j].tracking_error) || axes[j].acceleration_code==0))
      throw std::invalid_argument("Brak potwierdzonych mikrokroków, tolerancji śledzenia lub kodu acc");
  }
  if(!any) throw std::invalid_argument("Brak napędów do wykonania programu");
  double previous=-1;
  for(const auto &point:p.points) {
    if(!std::isfinite(point.seconds) || point.seconds<=previous) throw std::invalid_argument("Nieciągły zegar programu");
    previous=point.seconds;
    for(unsigned j=0;j<6;++j) if(!std::isfinite(point.positions[j]) || !std::isfinite(point.velocities[j]) || !std::isfinite(point.accelerations[j]))
      throw std::invalid_argument("Nieskończona próbka programu");
  }
  for(unsigned j=0;j<6;++j) if(p.participating[j] && (std::abs(p.points.front().velocities[j])>1e-8 ||
      std::abs(p.points.back().velocities[j])>1e-8)) throw std::invalid_argument("PTP wymaga postoju na obu końcach");
  // Otoczka Bernsteina ogranicza pozycję i obie pochodne na CAŁYM
  // przedziale quintic, również gdy węzły dzieli mniej niż 1 ms.
  for(size_t k=1;k<p.points.size();++k) {
    const auto &left=p.points[k-1],&right=p.points[k];const double h=right.seconds-left.seconds;
    for(unsigned j=0;j<6;++j) if(p.participating[j]) {
      const auto &a=axes[j];
      const std::array<double,6> b{left.positions[j],left.positions[j]+left.velocities[j]*h/5,
        left.positions[j]+2*left.velocities[j]*h/5+left.accelerations[j]*h*h/20,
        right.positions[j]-2*right.velocities[j]*h/5+right.accelerations[j]*h*h/20,
        right.positions[j]-right.velocities[j]*h/5,right.positions[j]};
      for(unsigned i=0;i<6;++i) {
        if(!std::isfinite(b[i]) || b[i]<a.lower-1e-9 || b[i]>a.upper+1e-9 ||
          (i<5 && std::abs(5*(b[i+1]-b[i])/h)>a.max_velocity+1e-6) ||
          (i<4 && std::abs(20*(b[i+2]-2*b[i+1]+b[i])/(h*h))>a.max_acceleration+1e-5))
          throw std::invalid_argument("Interpolacja nie mieści się w limitach osi");
      }
    }
  }
  // Weryfikacja interpolacji, także pomiędzy węzłami. Dane pochodzą z planera,
  // ale granice sprzętu są ponownie sprawdzane przez właściciela CAN.
  std::array<double,6> peaks{}, acceleration_peaks{};
  for(double t=0;t<p.duration+.0005;t+=.001) {
    const auto q=program_reference(p,std::min(t,p.duration));
    for(unsigned j=0;j<6;++j) if(p.participating[j]) {
      const auto &a=axes[j];
      if(q.positions[j]<a.lower-1e-9 || q.positions[j]>a.upper+1e-9 ||
          std::abs(q.velocities[j])>a.max_velocity+1e-6 || std::abs(q.accelerations[j])>a.max_acceleration+1e-5)
        throw std::invalid_argument("Trajektoria przekracza limity napędu J"+std::to_string(j+1));
      peaks[j]=std::max(peaks[j],std::abs(q.velocities[j]));
      acceleration_peaks[j]=std::max(acceleration_peaks[j],std::abs(q.accelerations[j]));
    }
  }
  // Próba odtworzenia całej trajektorii pojedynczym trapezem na napęd.
  // Dopuszczenie wymaga zgodności toru i wspólnych faz po kwantyzacji.
  bool native=true; std::array<NativeProfile,6> profiles{};
  ProgramFrame native_frame; double first_ramp=-1,first_finish=-1;
  for(unsigned j=0;j<6;++j) if(p.participating[j]) {
    const auto &a=axes[j];const double scale=speed_scale(microsteps[j]);
    const double shaft_to_joint=2*pi/(60*a.gear_ratio);
    const auto ticks=CanBusManager::joint_to_position(a,p.target[j]);
    const double distance=std::abs(CanBusManager::position_to_joint(a,ticks)-p.start[j]);
    const int rpm=std::max(1,static_cast<int>(std::llround(peaks[j]/shaft_to_joint/scale)));
    const double v=rpm*scale*shaft_to_joint;
    double best=1e100;unsigned best_code=a.acceleration_code; NativeProfile best_profile;
    for(unsigned acc=1;acc<=255;++acc) {
      const double acceleration=scale*shaft_to_joint/((256-acc)*.00005);
      if(acceleration>a.max_acceleration+1e-9 || v>a.max_velocity+1e-9 || rpm>3000) continue;
      const double ramp=std::min(v/acceleration,std::sqrt(distance/acceleration));
      const double speed=acceleration*ramp,cruise=speed>0 ? std::max(0.,(distance-acceleration*ramp*ramp)/speed) : 0.;
      const double finish=2*ramp+cruise,error=std::abs(finish-p.duration);
      if(error<best) { best=error;best_code=acc;best_profile={distance,speed,acceleration,ramp,cruise,finish}; }
    }
    profiles[j]=best_profile;
    if(distance>1e-12) {
      if(best>dt || (first_ramp>=0 && (std::abs(best_profile.ramp-first_ramp)>dt || std::abs(best_profile.finish-first_finish)>dt))) native=false;
      if(first_ramp<0) { first_ramp=best_profile.ramp;first_finish=best_profile.finish; }
    }
    native_frame.payloads[j]=absolute_position_request(a.address,ticks,std::min(rpm,3000),best_code);
  }
  if(native) for(double t=0;t<p.duration+.0005;t+=.001) {
    const auto expected=program_reference(p,std::min(t,p.duration));
    for(unsigned j=0;j<6;++j) if(p.participating[j]) {
      const double q=p.start[j]+(p.target[j]>=p.start[j] ? 1 : -1)*native_position(profiles[j],t);
      if(std::abs(q-expected.positions[j])>axes[j].tracking_error/4) native=false;
    }
  }
  if(native) { p.mode="native_ptp";p.frames={std::move(native_frame)};return p; }
  if(std::abs(std::round(p.duration/dt)*dt-p.duration)>1e-8)
    throw std::invalid_argument("Próbkowany PTP wymaga czasu końca na siatce cyklu CAN");
  const auto frame_count=static_cast<size_t>(std::llround(p.duration/dt));
  p.mode="streamed"; p.frames.clear();p.frames.reserve(frame_count);
  auto previous_q=p.start;
  for(size_t k=1;k<=frame_count;++k) {
    ProgramFrame frame;frame.seconds=(k-1)*dt;
    const auto point=program_reference(p,std::min(k*dt,p.duration));
    for(unsigned j=0;j<6;++j) if(p.participating[j]) {
      const auto &a=axes[j];const double scale=speed_scale(microsteps[j]);
      const double shaft=std::max(std::abs(point.velocities[j]),std::abs(point.positions[j]-previous_q[j])/dt)*a.gear_ratio*60/(2*pi);
      const double requested=shaft/scale,maximum=std::min(3000.,std::floor(a.max_velocity*a.gear_ratio*60/(2*pi)/scale));
      const auto ticks=CanBusManager::joint_to_position(a,point.positions[j]);
      const bool moving=ticks!=CanBusManager::joint_to_position(a,previous_q[j]);
      if(maximum<1 || requested>maximum+1e-6)
        throw std::invalid_argument("Odcinek nieprzedstawialny w RPM MKS; zmień profil ruchu");
      const auto rpm=static_cast<unsigned>(moving ? std::clamp(std::round(requested),1.,maximum) : 1.);
      const double rounding_error=std::abs(rpm-requested)*scale*2*pi/(60*a.gear_ratio)*dt;
      if(moving && rounding_error>a.tracking_error/4) throw std::invalid_argument("Kwantyzacja RPM przekracza tolerancję osi");
      if(point.velocities[j]*(point.positions[j]-previous_q[j])<-1e-12) throw std::invalid_argument("Zmiana kierunku wewnątrz próbki");
      const double acceleration=scale*2*pi/(60*a.gear_ratio)/((256-a.acceleration_code)*.00005);
      if(acceleration>a.max_acceleration+1e-9 || acceleration+1e-6<acceleration_peaks[j])
        throw std::invalid_argument("Kod acc nie odtwarza przyspieszenia trajektorii w limitach osi; dostosuj profil");
      frame.payloads[j]=absolute_position_request(a.address,ticks,rpm,a.acceleration_code);
    }
    p.frames.push_back(std::move(frame));previous_q=point.positions;
  }
  return p;
}
}
