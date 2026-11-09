#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace robot_kinematics {
struct ScalarSample {
  double position{}, velocity{}, acceleration{}, jerk{};
  bool done{};
};
// Analityczne fazy stałego jerku. Końcowy cel nie wynika z sumowania kroków 10 ms.
class JerkProfile {
 public:
  static JerkProfile move(double velocity,double acceleration,double jerk,double distance,double dt=.01) {
    validate(velocity,acceleration,jerk);
    if(!std::isfinite(distance) || distance<0 || !std::isfinite(dt) || dt<=0) throw std::invalid_argument("Nieprawidłowa droga/cykl S-Curve");
    JerkProfile p;
    const double tj0=std::min(acceleration/jerk,std::sqrt(velocity/jerk));
    const double tj=std::max(dt,std::ceil(tj0/dt-1e-12)*dt);
    const double ta=std::ceil(std::max(0.,velocity/acceleration-tj0)/dt-1e-12)*dt;
    const double ramp=2*tj+ta;
    const double peak=distance>0 ? std::min(velocity,distance/ramp) : velocity;
    const double effective_jerk=peak/(tj*(tj+ta));
    p.add(tj,effective_jerk);p.add(ta,0);p.add(tj,-effective_jerk);
    if(distance==0) {p.continuous_=true;p.braking_at_=INFINITY;return p;}
    p.add(std::max(0.,distance/peak-ramp),0);p.braking_at_=p.duration_;
    p.add(tj,-effective_jerk);p.add(ta,0);p.add(tj,effective_jerk);
    p.end_.position=distance;p.end_.velocity=p.end_.acceleration=p.end_.jerk=0;
    return p;
  }
  static JerkProfile stop(double velocity,double acceleration,double amax,double jerk) {
    validate(std::max(velocity,1e-30),amax,jerk);
    if(!std::isfinite(velocity) || velocity<0 || !std::isfinite(acceleration) || std::abs(acceleration)>amax+1e-9)
      throw std::invalid_argument("Nieprawidłowy stan początkowy hamowania S-Curve");
    JerkProfile p;p.end_.velocity=velocity;p.end_.acceleration=acceleration;
    if(velocity<1e-14 && std::abs(acceleration)<1e-12) {p.end_={};return p;}
    double b=std::min(amax,std::sqrt(jerk*velocity+.5*acceleration*acceleration));
    if(b+acceleration < -1e-9) throw std::runtime_error("Brak monotonicznej drogi hamowania S-Curve");
    const double plateau=std::max(0.,(velocity+(acceleration*acceleration-2*b*b)/(2*jerk))/b);
    p.add(std::max(0.,(acceleration+b)/jerk),-jerk);p.add(plateau,0);p.add(b/jerk,jerk);
    p.end_.velocity=p.end_.acceleration=p.end_.jerk=0;
    return p;
  }
  ScalarSample sample(double t) const {
    if(!std::isfinite(t) || t<0) throw std::invalid_argument("Nieprawidłowy czas S-Curve");
    for(const auto &segment:segments_) if(t<segment.begin+segment.duration) return integrate(segment.initial,t-segment.begin,segment.jerk);
    auto result=end_;
    if(continuous_) {result.position+=end_.velocity*std::max(0.,t-duration_);result.jerk=0;result.done=false;}
    else result.done=true;
    return result;
  }
  double duration() const {return duration_;}
  double braking_at() const {return braking_at_;}
 private:
  struct Segment {double begin,duration,jerk;ScalarSample initial;};
  static void validate(double v,double a,double j) {
    if(!std::isfinite(v) || v<=0 || !std::isfinite(a) || a<=0 || !std::isfinite(j) || j<=0)
      throw std::invalid_argument("S-Curve wymaga dodatnich skończonych v/a/j");
  }
  static ScalarSample integrate(ScalarSample s,double t,double j) {
    s.position+=s.velocity*t+.5*s.acceleration*t*t+j*t*t*t/6;
    s.velocity+=s.acceleration*t+.5*j*t*t;s.acceleration+=j*t;s.jerk=j;return s;
  }
  void add(double time,double jerk) {
    if(time<=1e-15)return;
    segments_.push_back({duration_,time,jerk,end_});end_=integrate(end_,time,jerk);duration_+=time;
  }
  std::vector<Segment> segments_;
  ScalarSample end_{};
  double duration_{},braking_at_{};
  bool continuous_{};
};
}
