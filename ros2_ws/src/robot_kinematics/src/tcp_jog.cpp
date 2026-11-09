#include "robot_kinematics/tcp_jog.hpp"
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_kinematics {
namespace { constexpr double dt=.010; }
TcpJog::TcpJog(Model model,const Joints &start,const Joints &vmax,const Joints &amax,TcpJogRequest request)
    : model_(std::move(model)),vmax_(vmax),amax_(amax),request_(request),origin_(model_.fk(start)) {
  if(request.axis>5 || (request.direction!=1 && request.direction!=-1) ||
      !std::isfinite(request.velocity) || request.velocity<=0 || !std::isfinite(request.acceleration) || request.acceleration<=0 ||
      !std::isfinite(request.step) || request.step<0 || request.velocity/request.acceleration>10 ||
      !std::isfinite(request.max_jerk) || request.max_jerk<=0 ||
      !std::isfinite(request.lookahead) || request.lookahead<0 || request.lookahead>.2)
    throw std::invalid_argument("Nieprawidłowy kierunek/profil TCP lub hamowanie dłuższe niż 10 s");
  if(!std::isfinite(request.position_tolerance) || request.position_tolerance<=0 || request.position_tolerance>1e-4 ||
      !std::isfinite(request.orientation_tolerance) || request.orientation_tolerance<=0 || request.orientation_tolerance>1e-3)
    throw std::invalid_argument("Tolerancje IK TCP poza zakresem (0, 0.1 mm] / (0, 0.001 rad]");
  for(unsigned j=0;j<6;++j) if(!std::isfinite(vmax[j]) || vmax[j]<=0 || !std::isfinite(amax[j]) || amax[j]<=0 ||
      start[j]<model_.lower()[j] || start[j]>model_.upper()[j]) throw std::invalid_argument("Nieprawidłowe limity/start JOG TCP");
  axis_=Eigen::Vector3d::Unit(request.axis%3)*request.direction;
  if(request.frame==TcpFrame::tool) axis_=origin_.rotation()*axis_;
  // Wspólne ograniczenie prędkości i przyspieszenia na początku drogi.
  Eigen::Matrix<double,6,1> twist=Eigen::Matrix<double,6,1>::Zero();
  if(request.axis<3) twist.head<3>()=axis_;else twist.tail<3>()=axis_;
  const auto tangent=model_.jacobian(start).completeOrthogonalDecomposition().solve(twist).eval();
  double velocity=request.velocity,acceleration=request.acceleration;
  for(unsigned j=0;j<6;++j) if(std::abs(tangent[j])>1e-12) {
    velocity=std::min(velocity,.9*vmax[j]/std::abs(tangent[j]));
    acceleration=std::min(acceleration,.9*amax[j]/std::abs(tangent[j]));
  }
  request_.acceleration=acceleration;
  profile_=JerkProfile::move(velocity,acceleration,request.max_jerk,request.step);
  current_.positions=current_.lookahead_positions=start;
  auto zero=current_;zero.done=true;zero.braking=true;brake_.push_back(zero);
}
Pose TcpJog::target(double progress) const {
  Pose pose=origin_;
  if(request_.axis<3) pose.translation()+=axis_*progress;
  else pose.linear()=Eigen::AngleAxisd(progress,axis_).toRotationMatrix()*origin_.rotation();
  return pose;
}
TcpJogSample TcpJog::point(const TcpJogSample &previous,const ScalarSample &scalar,const std::function<bool()> &cancelled) const {
  if(cancelled && cancelled()) throw std::runtime_error("Przekroczony czas obliczeń TCP");
  TcpJogSample out;out.speed=scalar.velocity;out.progress=scalar.position;
  out.scalar_acceleration=scalar.acceleration;out.scalar_jerk=scalar.jerk;
  if(request_.step>0 && out.progress>request_.step+1e-10) throw std::runtime_error("Droga hamowania przekracza impuls TCP");
  const auto solution=model_.solve(target(out.progress),previous.positions,{},request_.position_tolerance,request_.orientation_tolerance,cancelled);
  if(!solution.success) throw std::runtime_error(solution.message);
  out.positions=solution.positions;
  Eigen::Matrix<double,6,1> twist=Eigen::Matrix<double,6,1>::Zero();
  if(request_.axis<3) twist.head<3>()=axis_;else twist.tail<3>()=axis_;
  const auto jac=model_.jacobian(out.positions);
  const Eigen::Matrix<double,6,1> tangent=jac.completeOrthogonalDecomposition().solve(twist);
  if(!tangent.allFinite() || (jac*tangent-twist).norm()>1e-5) throw std::runtime_error("Kierunek TCP niedostępny przy osobliwości");
  for(unsigned j=0;j<6;++j) {
    out.velocities[j]=tangent[j]*scalar.velocity;
    out.accelerations[j]=(out.velocities[j]-previous.velocities[j])/dt;
    if(std::abs(out.positions[j]-previous.positions[j])>vmax_[j]*dt+1e-9 ||
        std::abs(out.velocities[j])>vmax_[j]+1e-9 || std::abs(out.accelerations[j])>amax_[j]+1e-7)
      throw std::runtime_error("Ograniczenie pozycji/prędkości/przyspieszenia J"+std::to_string(j+1));
  }
  return out;
}
std::deque<TcpJogSample> TcpJog::braking(const TcpJogSample &from,const std::function<bool()> &cancelled) const {
  std::deque<TcpJogSample> result;auto previous=from;
  const auto profile=JerkProfile::stop(from.speed,from.scalar_acceleration,request_.acceleration,request_.max_jerk);
  if(profile.duration()>10) throw std::runtime_error("Zbyt długa droga hamowania TCP");
  const auto count=std::max(1u,static_cast<unsigned>(std::ceil(profile.duration()/dt)));
  for(unsigned i=1;i<=count;++i) {
    auto scalar=profile.sample(std::min(i*dt,profile.duration()));scalar.position+=from.progress;
    auto next=point(previous,scalar,cancelled);next.braking=true;next.done=i==count;
    next.remaining=std::max(0.,profile.duration()-i*dt);
    result.push_back(next);previous=next;
  }
  return result;
}
void TcpJog::set_lookahead() {
  current_.lookahead_positions=current_.positions;
  if(!brake_.empty() && request_.lookahead>0) {
    const auto index=std::min(brake_.size()-1,static_cast<std::size_t>(std::max(0.,std::ceil(request_.lookahead/dt)-1)));
    current_.lookahead_positions=brake_[index].positions;
  }
}
TcpJogSample TcpJog::next(bool stop,const std::function<bool()> &cancelled) {
  stopping_=stopping_ || stop;
  if(!stopping_) {
    try {
      double next_time=time_+dt;
      if(request_.step>0 && std::abs(next_time-profile_.duration())<1e-10)next_time=profile_.duration();
      const auto scalar=profile_.sample(next_time);
      auto next=point(current_,scalar,cancelled);
      auto path=braking(next,cancelled);
      next.remaining=path.size()*dt;
      next.braking=time_+dt>=profile_.braking_at();next.done=scalar.done;
      current_=next;time_=next_time;brake_=std::move(path);set_lookahead();return current_;
    } catch(const std::runtime_error &e) {
      stop_reason_=e.what();
      if(cancelled && cancelled()) {
        stop_reason_="Przekroczony budżet cyklu obliczeń TCP";
        if(current_.progress==0 && current_.speed==0)return current_;
      }
    }
    // Brak miejsca na kolejny krok: hamowanie po sprawdzonej wcześniej drodze.
    stopping_=true;
  }
  if(!brake_.empty()) {current_=brake_.front();brake_.pop_front();}
  else {current_.velocities.fill(0);current_.accelerations.fill(0);current_.done=current_.braking=true;current_.remaining=0;}
  set_lookahead();return current_;
}
}
