#include "robot_motion_planner/planner.hpp"
#include "mks_servo_can/motion_sample.hpp"
#include "mks_servo_can/bus_manager.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <Eigen/SVD>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace robot_motion_planner {
namespace {
constexpr double cycle=.02;
constexpr size_t max_points=10000;
constexpr double wrist_slow_velocity=5*3.141592653589793/180;
double seconds(const builtin_interfaces::msg::Duration &duration) { return duration.sec+duration.nanosec*1e-9; }
builtin_interfaces::msg::Duration duration(double t) {
  builtin_interfaces::msg::Duration result; const auto ns=static_cast<int64_t>(std::llround(t*1e9));
  result.sec=ns/1000000000; result.nanosec=ns%1000000000; return result;
}
struct Sample { robot_kinematics::Joints q{},v{},a{}; };
struct TcpRates { Eigen::Vector3d v,a,w; };
TcpRates tcp_rates(const robot_kinematics::Model &model,const Sample &s) {
  const auto jac=model.jacobian(s.q);
  const Eigen::Map<const Eigen::Matrix<double,6,1>> v(s.v.data()),a(s.a.data());
  TcpRates out{jac.topRows<3>()*v,jac.topRows<3>()*a,jac.bottomRows<3>()*v};
  if(v.squaredNorm()>1e-20) {
    constexpr double h=1e-4;auto plus=s.q,minus=s.q;
    for(unsigned j=0;j<6;++j) { plus[j]+=h*s.v[j];minus[j]-=h*s.v[j]; }
    out.a+=(model.jacobian(plus).topRows<3>()-model.jacobian(minus).topRows<3>())*v/(2*h);
  }
  return out;
}
double tcp_excess(const robot_kinematics::Model &model,const Sample &s,const TcpLimits &limits) {
  if(limits.velocity<=0) return 1.;
  const auto rates=tcp_rates(model,s);
  return std::max({1.,rates.v.norm()/limits.velocity,rates.w.norm()/limits.angular_velocity,
                   std::sqrt(rates.a.norm()/limits.acceleration)});
}
Sample sample(const trajectory_msgs::msg::JointTrajectory &trajectory, double t, size_t &index) {
  const auto &points=trajectory.points;
  while(index+1<points.size()-1 && seconds(points[index+1].time_from_start)<t) ++index;
  const auto &left=points[index], &right=points[std::min(index+1,points.size()-1)];
  const double dt=seconds(right.time_from_start)-seconds(left.time_from_start);
  Sample out;
  for(unsigned j=0;j<6;++j) {
    if(dt<=0) { out.q[j]=left.positions[j]; continue; }
    const double s=std::clamp((t-seconds(left.time_from_start))/dt,0.,1.);
    const double c0=left.positions[j], c1=left.velocities[j]*dt, c2=left.accelerations[j]*dt*dt/2;
    const double d=right.positions[j]-c0, v=right.velocities[j]*dt, a=right.accelerations[j]*dt*dt;
    const double c3=10*d-6*c1-3*c2-4*v+a/2, c4=-15*d+8*c1+3*c2+7*v-a, c5=6*d-3*c1-c2-3*v+a/2;
    out.q[j]=c0+s*(c1+s*(c2+s*(c3+s*(c4+s*c5))));
    out.v[j]=(c1+s*(2*c2+s*(3*c3+s*(4*c4+s*5*c5))))/dt;
    out.a[j]=(2*c2+s*(6*c3+s*(12*c4+s*20*c5)))/(dt*dt);
  }
  return out;
}
std::array<double,6> local_velocity_limits(const robot_kinematics::Model &model, const Sample &s,
                                         const std::array<double,6> &requested) {
  auto limits=requested;
  if(std::abs(s.v[3])<=wrist_slow_velocity && std::abs(s.v[5])<=wrist_slow_velocity) return limits;
  const auto blend=wrist_singularity_blend(model,s.q);
  for(unsigned j : {3u,5u}) if(requested[j]>wrist_slow_velocity)
    limits[j]=wrist_slow_velocity+(requested[j]-wrist_slow_velocity)*blend;
  return limits;
}
void stretch_time(trajectory_msgs::msg::JointTrajectory &trajectory, double factor) {
  for(auto &point:trajectory.points) {
    point.time_from_start=duration(seconds(point.time_from_start)*factor);
    for(unsigned j=0;j<6;++j) { point.velocities[j]/=factor; point.accelerations[j]/=factor*factor; }
  }
}
void limit_path_speed(trajectory_msgs::msg::JointTrajectory &trajectory, const robot_kinematics::Model &model,
                const std::array<double,6> &vmax, const std::array<double,6> &amax, const TcpLimits &tcp,
                const std::function<void()> &check) {
  const auto total=seconds(trajectory.points.back().time_from_start);
  if(total<=0 || (tcp.velocity<=0 && vmax[3]<=wrist_slow_velocity && vmax[5]<=wrist_slow_velocity)) return;
  // Parametrem drogi pozostaje czas starej trajektorii s. Nowy zegar ma szybkość
  // u=ds/dt <= 1. Pozycje drogi się nie zmieniają; q'=q_s*u,
  // q''=q_ss*u²+q_s*du/dt. Ograniczamy również drugi składnik przy hamowaniu.
  const size_t count=std::min(max_points,std::max<size_t>(2,std::ceil(total/.01)+1));
  const double ds=total/(count-1);
  std::vector<Sample> samples; samples.reserve(count);
  std::vector<double> rate2(count,1.),budget(count,1e12);
  size_t cursor=0; bool slowing=false;
  for(size_t k=0;k<count;++k) {
    check(); samples.push_back(sample(trajectory,k*ds,cursor)); const auto &s=samples.back();
    const auto limits=local_velocity_limits(model,s,vmax);
    for(unsigned j=0;j<6;++j) {
      if(std::abs(s.v[j])>1e-10) {
        // Rezerwa na interpolację pomiędzy węzłami, tylko tam, gdzie limit zwalnia ruch.
        if(limits[j]<vmax[j]-1e-10) rate2[k]=std::min(rate2[k],std::pow(.995*limits[j]/std::abs(s.v[j]),2));
        budget[k]=std::min(budget[k],std::max(0.,.999*amax[j]-std::abs(s.a[j]))/std::abs(s.v[j]));
      }
    }
    if(tcp.velocity>0) {
      const auto rates=tcp_rates(model,s);
      if(rates.v.norm()>1e-10) rate2[k]=std::min(rate2[k],std::pow(.995*tcp.velocity/rates.v.norm(),2));
      if(rates.w.norm()>1e-10) rate2[k]=std::min(rate2[k],std::pow(.995*tcp.angular_velocity/rates.w.norm(),2));
      // Rezerwa dla przyspieszenia powodowanego zmianą zegara drogi.
      if(rates.a.norm()>1e-10) rate2[k]=std::min(rate2[k],.9*tcp.acceleration/rates.a.norm());
      if(rates.v.norm()>1e-10) budget[k]=std::min(budget[k],
        std::max(0.,.999*tcp.acceleration-rates.a.norm()*rate2[k])/rates.v.norm());
    }
    slowing=slowing || rate2[k]<1.-1e-10;
  }
  if(!slowing && tcp.velocity<=0) return;
  // Wyprzedzające hamowanie i ograniczenie przyspieszania po opuszczeniu strefy.
  // |d(u²)/ds| <= 2*budget; obie strony dostają wspólny budżet interwału.
  for(size_t k=1;k<count;++k) rate2[k]=std::min(rate2[k],rate2[k-1]+2*std::min(budget[k-1],budget[k])*ds);
  for(size_t k=count-1;k>0;--k) rate2[k-1]=std::min(rate2[k-1],rate2[k]+2*std::min(budget[k-1],budget[k])*ds);
  std::vector<double> acceleration(count-1);
  for(size_t k=0;k+1<count;++k) acceleration[k]=(rate2[k+1]-rate2[k])/(2*ds);
  trajectory_msgs::msg::JointTrajectory output; output.joint_names=trajectory.joint_names; output.header=trajectory.header;
  double t=0;
  for(size_t k=0;k<count;++k) {
    check(); const double u=std::sqrt(rate2[k]);
    const double alpha=k==0 ? acceleration.front() : k+1==count ? acceleration.back() : (acceleration[k-1]+acceleration[k])/2;
    trajectory_msgs::msg::JointTrajectoryPoint point; point.positions.resize(6);point.velocities.resize(6);point.accelerations.resize(6);
    for(unsigned j=0;j<6;++j) {
      point.positions[j]=samples[k].q[j]; point.velocities[j]=samples[k].v[j]*u;
      point.accelerations[j]=samples[k].a[j]*rate2[k]+samples[k].v[j]*alpha;
    }
    if(k) t+=2*ds/(std::sqrt(rate2[k-1])+u);
    if(t>200) throw std::runtime_error("Plan po ograniczeniu prędkości TCP/nadgarstka przekracza 200 s");
    point.time_from_start=duration(t); output.points.push_back(std::move(point));
  }
  trajectory=std::move(output);
  // Interpolator JTC używa q/v/a, więc sprawdzamy cały wynik po zmianie zegara.
  double correction=1.; cursor=0;
  for(double t=0;t<=seconds(trajectory.points.back().time_from_start);t+=.001) {
    check(); const auto s=sample(trajectory,t,cursor); const auto limits=local_velocity_limits(model,s,vmax);
    for(unsigned j=0;j<6;++j) correction=std::max({correction,std::abs(s.v[j])/limits[j],std::sqrt(std::abs(s.a[j])/amax[j])});
    correction=std::max(correction,tcp_excess(model,s,tcp));
  }
  if(correction>1.+1e-8) stretch_time(trajectory,correction*1.005);
  const double finish=seconds(trajectory.points.back().time_from_start);
  stretch_time(trajectory,std::ceil(finish/cycle)*cycle/finish);
}
bool near_path(const Pose &pose, const std::vector<Pose> &path, double p_tol, double r_tol) {
  for(size_t i=1;i<path.size();++i) {
    const auto d=path[i].translation()-path[i-1].translation();
    double t;
    // Resztkowy błąd FK/IK przy samym obrocie nie może wyznaczać postępu
    // orientacji przez projekcję na mikrometrowy wektor translacji.
    if(d.squaredNorm()>std::max(1e-16,p_tol*p_tol)) t=std::clamp((pose.translation()-path[i-1].translation()).dot(d)/d.squaredNorm(),0.,1.);
    else {
      const Eigen::AngleAxisd delta(path[i].rotation()*path[i-1].rotation().transpose());
      const Eigen::AngleAxisd actual(pose.rotation()*path[i-1].rotation().transpose());
      t=delta.angle()>1e-9 ? std::clamp(actual.axis().dot(delta.axis())*actual.angle()/delta.angle(),0.,1.) : 0;
    }
    const auto nearest=robot_kinematics::interpolate(path[i-1],path[i],t);
    if((nearest.translation()-pose.translation()).norm()<=p_tol && Eigen::AngleAxisd(nearest.rotation()*pose.rotation().transpose()).angle()<=r_tol) return true;
  }
  return false;
}
std::string escape(std::string s) {
  for(const auto &pair : {std::pair<std::string,std::string>{"&","&amp;"},{"\"","&quot;"},{"<","&lt;"},{">","&gt;"}}) {
    size_t p=0; while((p=s.find(pair.first,p))!=std::string::npos) { s.replace(p,pair.first.size(),pair.second); p+=pair.second.size(); }
  } return s;
}
}
double wrist_singularity_blend(const robot_kinematics::Model &model, const robot_kinematics::Joints &positions) {
  // Wyłącznie część kątowa J4–J6: bez mieszania metrów z radianami i bez
  // zakładania osi obrotu z samego kąta J5. Kierunki pochodzą z aktualnego URDF.
  const Eigen::Matrix3d wrist=model.jacobian(positions).bottomRightCorner<3,3>();
  const auto values=wrist.jacobiSvd().singularValues().eval();
  const double ratio=values[0]>1e-12 ? values[2]/values[0] : 0.;
  // Progi podglądu: dla prostopadłego nadgarstka odpowiadają J5=5° i 30°.
  constexpr double near=.04366094290851206,far=.2679491924311227;
  const double x=std::clamp((ratio-near)/(far-near),0.,1.);
  return std::clamp(x*x*x*(10+x*(-15+6*x)),0.,1.);  // ciągłe pochodne na progach
}
Pose pose_from_message(const geometry_msgs::msg::Pose &message) {
  Eigen::Quaterniond q(message.orientation.w,message.orientation.x,message.orientation.y,message.orientation.z);
  if(!q.coeffs().allFinite() || q.norm()<1e-9 || std::abs(q.norm()-1)>.01) throw std::invalid_argument("Nieprawidłowy kwaternion celu");
  Pose result=Pose::Identity(); result.translation()=Eigen::Vector3d(message.position.x,message.position.y,message.position.z);
  result.linear()=q.normalized().toRotationMatrix();
  if(!result.matrix().allFinite()) throw std::invalid_argument("Nieprawidłowa pozycja TCP");
  return result;
}
geometry_msgs::msg::Pose pose_message(const Pose &pose) {
  geometry_msgs::msg::Pose result; result.position.x=pose.translation().x(); result.position.y=pose.translation().y(); result.position.z=pose.translation().z();
  const Eigen::Quaterniond q(pose.rotation()); result.orientation.x=q.x(); result.orientation.y=q.y(); result.orientation.z=q.z(); result.orientation.w=q.w(); return result;
}
void validate_context(const Context &c) {
  if(c.model_id!=robot_kinematics::model_id(c.urdf)) throw std::invalid_argument("Identyfikator modelu różni się od URDF");
  if(c.configuration_revision.empty() || c.configuration_revision.size()>128) throw std::invalid_argument("Brak rewizji konfiguracji");
  for(unsigned i=0;i<6;++i) {
    if(!c.known[i] || !std::isfinite(c.seed[i])) throw std::invalid_argument("Uzupełnij jawnie pozycje zamrożonych osi; nieznane zero nie jest pomiarem");
    if(!std::isfinite(c.lower[i]) || !std::isfinite(c.upper[i]) || c.lower[i]>=c.upper[i] || c.seed[i]<c.lower[i] || c.seed[i]>c.upper[i])
      throw std::invalid_argument("Pozycja startowa lub zakres osi poza limitami");
    if(!std::isfinite(c.max_velocity[i]) || c.max_velocity[i]<=0 || !std::isfinite(c.max_acceleration[i]) || c.max_acceleration[i]<=0)
      throw std::invalid_argument("Brak dodatnich limitów prędkości/przyspieszenia do planowania");
  }
}
static Plan compute_impl(const Context &c,const std::vector<Pose> &targets,double p_tol,double r_tol,double v_scale,double a_scale,
             const std::array<int64_t,6> &codes,const std::function<bool()> &cancelled,const std::function<void(float,const std::string&)> &progress,
             const robot_kinematics::Joints *joint_target=nullptr,bool ptp=false,const TcpLimits &tcp={}) {
  validate_context(c);
  if(targets.empty() || targets.size()>32 || !std::isfinite(p_tol) || p_tol<=0 || p_tol>.05 || !std::isfinite(r_tol) || r_tol<=0 || r_tol>.5 ||
     !std::isfinite(v_scale) || v_scale<=0 || v_scale>1 || !std::isfinite(a_scale) || a_scale<=0 || a_scale>1) throw std::invalid_argument("Nieprawidłowe cele, tolerancje lub skala czasu ruchu");
  robot_kinematics::Model kinematics(c.urdf,c.lower,c.upper);
  auto check=[&]() { if(cancelled && cancelled()) throw std::runtime_error("Obliczenia anulowane lub kontekst zmieniony"); };
  auto report=[&](float value,const std::string &phase) { check(); if(progress) progress(value,phase); };
  std::vector<Pose> path{kinematics.fk(c.seed)}; path.insert(path.end(),targets.begin(),targets.end());
  std::vector<robot_kinematics::Joints> positions{c.seed}; auto previous=c.seed;
  if(ptp) positions.push_back(*joint_target);
  for(size_t i=1;!ptp && i<path.size();++i) {
    const auto distance=(path[i].translation()-path[i-1].translation()).norm();
    const double angle=Eigen::AngleAxisd(path[i].rotation()*path[i-1].rotation().transpose()).angle();
    const size_t steps=std::max<size_t>(1,std::ceil(std::max(distance/.001,angle/.01)));
    if(positions.size()+steps>2000) throw std::invalid_argument("Droga przekracza 2000 punktów IK; podziel program przed obliczeniami");
    for(size_t step=1;step<=steps;++step) {
      check(); const auto solution=kinematics.solve(robot_kinematics::interpolate(path[i-1],path[i],double(step)/steps),previous,c.locked,
                                                    std::min(1e-5,p_tol/10),std::min(1e-4,r_tol/10),cancelled);
      if(!solution.success) throw std::runtime_error(solution.message+"; odcinek "+std::to_string(i)+", punkt "+std::to_string(step));
      for(unsigned j=0;j<6;++j) if(std::abs(solution.positions[j]-previous[j])>.25) throw std::runtime_error("Przeskok gałęzi IK; zagęść lub zmień drogę");
      previous=solution.positions; positions.push_back(previous);
    }
    report(.45f*float(i)/targets.size(),"IK pełnej drogi");
  }
  if(joint_target && !ptp) {
    for(unsigned i=0;i<6;++i) if(std::abs(positions.back()[i]-(*joint_target)[i])>3.141592653589793/1800)
      throw std::runtime_error("LIN prowadzi do innej konfiguracji końcowej; wybierz PTP lub zmień cel");
    positions.back()=*joint_target;
  }
  auto parsed=urdf::parseURDF(c.urdf); if(!parsed) throw std::invalid_argument("Nieprawidłowy URDF");
  // MoveIt nie może zawinąć wieloobrotowego continuous do najkrótszego obrotu.
  for(unsigned i=0;i<6;++i) {
    auto joint=parsed->joints_.at("joint_"+std::to_string(i+1)); joint->type=urdf::Joint::REVOLUTE;
    if(!joint->limits) joint->limits=std::make_shared<urdf::JointLimits>();
    joint->limits->lower=c.lower[i]; joint->limits->upper=c.upper[i]; joint->limits->velocity=c.max_velocity[i];
  }
  auto srdf=std::make_shared<srdf::Model>();
  if(!srdf->initString(*parsed,"<robot name=\""+escape(parsed->getName())+"\"><group name=\"arm\"><chain base_link=\""+escape(kinematics.base())+"\" tip_link=\""+escape(kinematics.tip())+"\"/></group></robot>"))
    throw std::runtime_error("Nie udało się zbudować grupy kinematycznej");
  auto model=std::make_shared<moveit::core::RobotModel>(parsed,srdf);
  robot_trajectory::RobotTrajectory trajectory(model,"arm");
  moveit::core::RobotState waypoint(model); waypoint.setToDefaultValues();
  std::unordered_map<std::string,double> velocity_limits,acceleration_limits;
  for(unsigned i=0;i<6;++i) { const auto name="joint_"+std::to_string(i+1); velocity_limits[name]=c.max_velocity[i]*v_scale; acceleration_limits[name]=c.max_acceleration[i]*a_scale; }
  for(const auto &q:positions) {
    for(unsigned i=0;i<6;++i) waypoint.setVariablePosition("joint_"+std::to_string(i+1),q[i]);
    waypoint.update(); trajectory.addSuffixWayPoint(waypoint,0);
  }
  report(.5,"TOTG: płynne połączenia i czas");
  trajectory_processing::TimeOptimalTrajectoryGeneration retimer(joint_target ? 0. : std::min(.001,p_tol/.5),cycle,1e-7);
  if(!retimer.computeTimeStamps(trajectory,velocity_limits,acceleration_limits)) throw std::runtime_error("TOTG odrzucił drogę");
  moveit_msgs::msg::RobotTrajectory message; trajectory.getRobotTrajectoryMsg(message);
  auto output=message.joint_trajectory;
  // Ustalona kolejność kontraktu ROS i kompilatora MKS.
  std::vector<unsigned> order;
  for(unsigned i=0;i<6;++i) { const auto name="joint_"+std::to_string(i+1); auto it=std::find(output.joint_names.begin(),output.joint_names.end(),name); if(it==output.joint_names.end()) throw std::runtime_error("Brak osi w wyniku TOTG"); order.push_back(it-output.joint_names.begin()); }
  for(auto &point:output.points) { auto old=point; for(unsigned i=0;i<6;++i) { point.positions[i]=old.positions[order[i]]; point.velocities[i]=old.velocities[order[i]]; point.accelerations[i]=old.accelerations[order[i]]; } }
  for(unsigned i=0;i<6;++i) output.joint_names[i]="joint_"+std::to_string(i+1);
  if(output.points.size()<2) {
    auto point=output.points.front(); point.time_from_start=duration(cycle); output.points.push_back(point);
  }
  double stretch=1.; size_t cursor=0;
  const auto initial_duration=seconds(output.points.back().time_from_start);
  if(initial_duration>200 || output.points.size()>max_points) throw std::runtime_error("Plan przekracza limit 200 s / 10000 punktów");
  for(double t=0;t<=initial_duration;t+=.001) {
    check(); const auto s=sample(output,t,cursor);
    for(unsigned i=0;i<6;++i) stretch=std::max({stretch,std::abs(s.v[i])/velocity_limits.at(output.joint_names[i]),std::sqrt(std::abs(s.a[i])/acceleration_limits.at(output.joint_names[i]))});
  }
  // Zapas również wtedy, gdy szczyt pochodnej leży między próbkami walidacji.
  stretch*=1.05;
  if(initial_duration>0) stretch=std::ceil(initial_duration*stretch/cycle)*cycle/initial_duration;
  stretch_time(output,stretch);
  std::array<double,6> vmax,amax;
  for(unsigned j=0;j<6;++j) { vmax[j]=velocity_limits.at(output.joint_names[j]); amax[j]=acceleration_limits.at(output.joint_names[j]); }
  report(.6,tcp.velocity>0 ? "Profil prędkości TCP i ograniczenia nadgarstka" : "Płynne zwalnianie J4/J6 przy osobliwości nadgarstka");
  limit_path_speed(output,kinematics,vmax,amax,tcp,check);
  const auto total=seconds(output.points.back().time_from_start);
  if(total>200) throw std::runtime_error("Plan po ograniczeniu pochodnych przekracza 200 s");
  report(.65,"Sprawdzenie interpolacji JTC i tolerancji TCP"); cursor=0;
  for(double t=0;t<=total+1e-9;t+=.005) {
    check(); const auto s=sample(output,std::min(t,total),cursor);
    const auto limits=local_velocity_limits(kinematics,s,vmax);
    if(tcp_excess(kinematics,s,tcp)>1.+1e-5) throw std::runtime_error("Interpolacja przekracza prędkość lub przyspieszenie TCP");
    for(unsigned i=0;i<6;++i) if(!std::isfinite(s.q[i]) || s.q[i]<c.lower[i]-1e-9 || s.q[i]>c.upper[i]+1e-9 ||
        std::abs(s.v[i])>limits[i]+1e-6 || std::abs(s.a[i])>acceleration_limits.at(output.joint_names[i])+1e-5 ||
        (c.locked[i] && std::abs(s.q[i]-c.seed[i])>1e-9)) throw std::runtime_error("Interpolacja narusza limit J"+std::to_string(i+1)+" t="+std::to_string(t)+" q="+std::to_string(s.q[i])+" v="+std::to_string(s.v[i])+" a="+std::to_string(s.a[i]));
    if(!ptp && !near_path(kinematics.fk(s.q),path,p_tol,r_tol)) throw std::runtime_error("Wygładzona droga przekracza tolerancję TCP; zmień punkty lub tolerancję");
  }
  Plan plan; plan.model_id=c.model_id; plan.configuration_revision=c.configuration_revision; plan.start_positions=c.seed; plan.locked=c.locked; plan.trajectory=output;
  plan.simulation_only=c.simulation_only; plan.simulated_start=c.simulated_start;
  plan.motion_type=joint_target ? (ptp ? "PTP" : "LIN") : "CARTESIAN";
  std::copy(output.points.back().positions.begin(),output.points.back().positions.end(),plan.target_positions.begin());
  plan.id=robot_kinematics::model_id(c.model_id+c.configuration_revision+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  plan.execution_ready=false; plan.execution_message="Podgląd: wykonanie wymaga odrębnego odbioru geometrii, kalibracji, synchronizacji i zatrzymania";
  report(.8,"Kompilacja podglądu F5 / ACK / SYNC");
  try {
    std::array<mks_servo_can::AxisConfig,6> axes;
    for(unsigned i=0;i<6;++i) {
      const auto &p=c.profiles[i];
      if(!p.configured || codes[i]<1 || codes[i]>255) throw std::runtime_error("Brak pełnego profilu osi lub jawnie potwierdzonego kodu MKS acc; trajektoria pozostaje podglądem");
      axes[i]={static_cast<uint16_t>(i+1),p.gear_ratio,double(p.direction),p.zero_offset_rad,p.lower_rad,p.upper_rad,p.velocity_rad_s,p.acceleration_rad_s2,0,static_cast<uint8_t>(codes[i])};
    }
    auto previous=c.seed; cursor=0;
    for(double t=0;t<=total+1e-9;t+=cycle) {
      check(); const auto s=sample(output,std::min(t,total),cursor); robot_control_interfaces::msg::MksPlanSample frame; frame.time_from_start=t;
      for(unsigned i=0;i<6;++i) {
        const auto motor=mks_servo_can::compile_motor_sample(axes[i],s.q[i],s.v[i],previous[i],cycle);
        frame.position_ticks[i]=motor.ticks; frame.speed_rpm[i]=motor.rpm; frame.acceleration_code[i]=motor.acceleration; frame.operation[i]=motor.operation;
        std::copy(motor.payload.begin(),motor.payload.end(),frame.payloads.begin()+i*8);
      }
      plan.mks_samples.push_back(frame); previous=s.q;
    }
    plan.mks_compatible=true; plan.mks_message="Przedstawialne F5 przy 50 Hz; ACK/SYNC i śledzenie wymagają odbioru sprzętu";
  } catch(const std::exception &e) { plan.mks_samples.clear(); plan.mks_message=e.what(); }
  report(1,"Plan gotowy do podglądu"); return plan;
}
Plan compute(const Context &c,const std::vector<Pose> &targets,double p_tol,double r_tol,double v_scale,double a_scale,
             const std::array<int64_t,6> &codes,const std::function<bool()> &cancelled,const std::function<void(float,const std::string&)> &progress) {
  return compute_impl(c,targets,p_tol,r_tol,v_scale,a_scale,codes,cancelled,progress);
}
Plan compute_physical_ptp(const Context &c,const robot_kinematics::Joints &target,const std::array<int64_t,6> &codes,
    const std::function<bool()> &cancelled,const std::function<void(float,const std::string&)> &progress) {
  validate_context(c);
  if(c.simulation_only) throw std::invalid_argument("Fizyczny PTP wymaga kontekstu pomiarowego");
  for(unsigned j=0;j<6;++j) {
    if(c.locked[j] || !std::isfinite(target[j]) || target[j]<c.lower[j] || target[j]>c.upper[j])
      throw std::invalid_argument("Cel PTP poza limitami lub zablokowana oś");
    if(c.simulated_start[j] ? !c.allow_missing_execution : !c.profiles[j].configured)
      throw std::invalid_argument("Założona pozycja wymaga trybu niepełnego zestawu; obecna oś wymaga profilu");
  }
  robot_kinematics::Model model(c.urdf,c.lower,c.upper);
  return compute_impl(c,{model.fk(target)},1e-4,3.141592653589793/1800,1,1,codes,cancelled,progress,&target,true);
}
Plan compute_preview(const Context &c,const robot_kinematics::Joints &target,uint8_t type,const std::array<int64_t,6> &codes,
             const std::function<bool()> &cancelled,const std::function<void(float,const std::string&)> &progress,const TcpLimits &tcp) {
  validate_context(c);
  if(!c.simulation_only || type>1) throw std::invalid_argument("Wymagany typ PTP/LIN i jawny tryb symulacji");
  const bool tcp_requested=tcp.velocity!=0 || tcp.acceleration!=0 || tcp.angular_velocity!=0;
  if(tcp_requested && (type!=1 || !std::isfinite(tcp.velocity) || tcp.velocity<=0 ||
      !std::isfinite(tcp.acceleration) || tcp.acceleration<=0 || !std::isfinite(tcp.angular_velocity) || tcp.angular_velocity<=0))
    throw std::invalid_argument("LIN wymaga dodatnich limitów posuwu, przyspieszenia i obrotu TCP");
  for(unsigned i=0;i<6;++i) if(c.locked[i] || !std::isfinite(target[i]) || target[i]<c.lower[i] || target[i]>c.upper[i])
    throw std::invalid_argument("Podgląd wymaga sześciu osi i celu w limitach");
  robot_kinematics::Model model(c.urdf,c.lower,c.upper);
  auto plan=compute_impl(c,{model.fk(target)},1e-4,3.141592653589793/1800,1,1,codes,cancelled,progress,&target,type==0,tcp);
  robot_kinematics::Joints end; std::copy(plan.trajectory.points.back().positions.begin(),plan.trajectory.points.back().positions.end(),end.begin());
  if((model.fk(end).translation()-model.fk(target).translation()).norm()>1e-4 ||
     Eigen::AngleAxisd(model.fk(end).rotation()*model.fk(target).rotation().transpose()).angle()>3.141592653589793/1800)
    throw std::runtime_error("Nieuzyskana dokładność końcowej pozy TCP");
  return plan;
}
}
