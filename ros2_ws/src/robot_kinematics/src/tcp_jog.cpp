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
      !std::isfinite(request.lookahead) || request.lookahead<0 || request.lookahead>.2)
    throw std::invalid_argument("Nieprawidłowy kierunek/profil TCP lub hamowanie dłuższe niż 10 s");
  if(!std::isfinite(request.position_tolerance) || request.position_tolerance<=0 || request.position_tolerance>1e-4 ||
      !std::isfinite(request.orientation_tolerance) || request.orientation_tolerance<=0 || request.orientation_tolerance>1e-3)
    throw std::invalid_argument("Tolerancje IK TCP poza zakresem (0, 0.1 mm] / (0, 0.001 rad]");
  for(unsigned j=0;j<6;++j) if(!std::isfinite(vmax[j]) || vmax[j]<=0 || !std::isfinite(amax[j]) || amax[j]<=0 ||
      start[j]<model_.lower()[j] || start[j]>model_.upper()[j]) throw std::invalid_argument("Nieprawidłowe limity/start JOG TCP");
  axis_=Eigen::Vector3d::Unit(request.axis%3)*request.direction;
  if(request.frame==TcpFrame::tool) axis_=origin_.rotation()*axis_;
  current_.positions=current_.lookahead_positions=start;
  auto zero=current_;zero.done=true;zero.braking=true;brake_.push_back(zero);
}
Pose TcpJog::target(double progress) const {
  Pose pose=origin_;
  if(request_.axis<3) pose.translation()+=axis_*progress;
  else pose.linear()=Eigen::AngleAxisd(progress,axis_).toRotationMatrix()*origin_.rotation();
  return pose;
}
TcpJogSample TcpJog::point(const TcpJogSample &previous,double speed,double distance,const std::function<bool()> &cancelled) const {
  if(cancelled && cancelled()) throw std::runtime_error("Przekroczony czas obliczeń TCP");
  TcpJogSample out;out.speed=speed;out.progress=previous.progress+distance;
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
    out.velocities[j]=tangent[j]*speed;
    out.accelerations[j]=(out.velocities[j]-previous.velocities[j])/dt;
    if(std::abs(out.positions[j]-previous.positions[j])>vmax_[j]*dt+1e-9 ||
        std::abs(out.velocities[j])>vmax_[j]+1e-9 || std::abs(out.accelerations[j])>amax_[j]+1e-7)
      throw std::runtime_error("Ograniczenie pozycji/prędkości/przyspieszenia J"+std::to_string(j+1));
  }
  return out;
}
std::deque<TcpJogSample> TcpJog::braking(const TcpJogSample &from,const std::function<bool()> &cancelled) const {
  std::deque<TcpJogSample> result;auto previous=from;
  do {
    const double speed=std::max(0.,previous.speed-request_.acceleration*dt);
    const double duration=std::min(dt,previous.speed/request_.acceleration);
    auto next=point(previous,speed,.5*(previous.speed+speed)*duration,cancelled);
    next.braking=true;next.done=speed==0;
    result.push_back(next);previous=next;
  } while(previous.speed>0 && result.size()<=1001);
  if(previous.speed>0) throw std::runtime_error("Zbyt długa droga hamowania TCP");
  const double duration=result.size()*dt;
  for(std::size_t i=0;i<result.size();++i) result[i].remaining=duration-(i+1)*dt;
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
    const double lo=std::max(0.,current_.speed-request_.acceleration*dt);
    const double hi=std::min(request_.velocity,current_.speed+request_.acceleration*dt);
    for(unsigned attempt=0;attempt<12;++attempt) {
      const double speed=lo+(hi-lo)*std::pow(.5,attempt);
      try {
        auto next=point(current_,speed,.5*(current_.speed+speed)*dt,cancelled);
        auto path=braking(next,cancelled);
        next.remaining=path.size()*dt;
        current_=next;brake_=std::move(path);set_lookahead();return current_;
      } catch(const std::runtime_error &e) {
        stop_reason_=e.what();
        if(cancelled && cancelled()) {
          stop_reason_="Przekroczony budżet cyklu obliczeń TCP";
          // Zimny solver lub wywłaszczenie przed pierwszą referencją: pozostajemy
          // w postoju i ponawiamy obliczenie w następnym cyklu. Po ruszeniu
          // przekroczenie budżetu zawsze wybiera zatwierdzone hamowanie.
          if(attempt==0 && current_.progress==0 && current_.speed==0) return current_;
          break;
        }
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
