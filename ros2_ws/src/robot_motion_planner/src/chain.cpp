// Ciąg ruchów PTP/LIN: przejścia w promieniu wokół punktu (krzywa Hermite'a 5. stopnia w przegubach),
// postoje (v=0) wewnątrz jednej trajektorii, czasy ruchów dla kursora robota. Opis: docs/programming.md.
#include "planner_detail.hpp"
#include "mks_servo_can/bus_manager.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

namespace robot_motion_planner {
using namespace detail;
namespace {
using Joints=robot_kinematics::Joints;
using ChainSegmentTimingMsg=robot_control_interfaces::msg::ChainSegmentTiming;
constexpr double cycle=mks_servo_can::kMotionCycleSeconds;
constexpr double pi=3.141592653589793;
constexpr double min_blend=.0005;          // m; mniejszy promień → postój
constexpr double max_reversal=170*pi/180;  // zawrócenie drogi przegubów → postój
constexpr size_t max_segment_points=2000, max_total_points=20000;
constexpr double totg_deviation=1e-4;     // rad

Joints lerp(const Joints &a,const Joints &b,double t) { Joints r; for(unsigned j=0;j<6;++j) r[j]=a[j]+(b[j]-a[j])*t; return r; }
double norm(const Joints &a,const Joints &b) { double s=0; for(unsigned j=0;j<6;++j) s+=(a[j]-b[j])*(a[j]-b[j]); return std::sqrt(s); }
double max_abs(const Joints &a,const Joints &b) { double s=0; for(unsigned j=0;j<6;++j) s=std::max(s,std::abs(a[j]-b[j])); return s; }

// Gęsta droga jednego ruchu: przeguby, TCP i długość toru TCP.
struct SegPath {
  std::vector<Joints> q; std::vector<Eigen::Vector3d> p; double length{}; Pose start,end;
  Joints at(double sigma) const {
    const size_t k=std::min(q.size()-2,static_cast<size_t>(std::floor(sigma)));
    return lerp(q[k],q[k+1],sigma-k);
  }
};
Joints joint_tangent(const SegPath &s,double sigma) {
  size_t k=std::min(s.q.size()-2,static_cast<size_t>(std::floor(sigma)));
  // Kierunek krawędzi zawierającej sigma (bez zerowych krawędzi).
  for(size_t i=k;i+1<s.q.size();++i) { if(norm(s.q[i+1],s.q[i])>1e-12) { k=i; break; } }
  Joints d; const double n=norm(s.q[k+1],s.q[k]);
  for(unsigned j=0;j<6;++j) d[j]=n>0 ? (s.q[k+1][j]-s.q[k][j])/n : 0;
  return d;
}
double angle(const Joints &a,const Joints &b) {
  double dot=0; for(unsigned j=0;j<6;++j) dot+=a[j]*b[j];
  return std::acos(std::clamp(dot,-1.,1.));
}

struct Label { int segment{}; bool blend{}; };
struct Vertex { Joints q; Label label; };

// Rzutowanie próbki trajektorii na drogę (łamana przegubów) z kursorem idącym tylko do przodu.
class Projector {
 public:
  explicit Projector(const std::vector<Vertex> &v):v_(v) {}
  void reset() { cursor_=0; }
  size_t edge(const Joints &q) {
    if(v_.size()<2) return 0;
    double best=1e300; size_t found=cursor_;
    for(size_t k=cursor_;k+1<v_.size() && k<cursor_+96;++k) {
      Joints d,w; double dd=0,dw=0;
      for(unsigned j=0;j<6;++j) { d[j]=v_[k+1].q[j]-v_[k].q[j]; w[j]=q[j]-v_[k].q[j]; dd+=d[j]*d[j]; dw+=d[j]*w[j]; }
      const double t=dd>0 ? std::clamp(dw/dd,0.,1.) : 0.;
      double dist=0; for(unsigned j=0;j<6;++j) dist+=std::pow(w[j]-t*d[j],2);
      if(dist<best-1e-18) { best=dist; found=k; }
    }
    cursor_=found; return found;
  }
  Label label(const Joints &q) { return v_[edge(q)].label; }
 private:
  const std::vector<Vertex> &v_; size_t cursor_{};
};

struct Corner { bool blend{}; double radius{}; double sigma_exit{},sigma_entry{}; std::vector<Joints> curve; Eigen::Vector3d point; };

std::string warning(size_t segment,const std::string &code,const std::string &text) {
  return std::to_string(segment)+"|"+code+"|"+text;
}
}

ChainResult compute_chain(const Context &c,const std::vector<ChainSegment> &input,double max_duration,
             const std::array<int64_t,6> &codes,const std::function<bool()> &cancelled,
             const std::function<void(float,const std::string&)> &progress) {
  validate_context(c);
  if(input.empty() || input.size()>64) throw std::invalid_argument("Ciąg wymaga 1–64 ruchów");
  if(!std::isfinite(max_duration) || max_duration<=0 || max_duration>200) throw std::invalid_argument("Maksymalny czas ciągu poza zakresem (0, 200] s");
  for(unsigned j=0;j<6;++j) {
    if(c.locked[j]) throw std::invalid_argument("Ciąg ruchów wymaga sześciu odblokowanych osi");
    if(!c.simulation_only && (c.simulated_start[j] || c.allow_missing_execution || !c.profiles[j].configured))
      throw std::invalid_argument("Fizyczny ciąg wymaga pomiaru i profilu wszystkich osi");
  }
  for(size_t i=0;i<input.size();++i) {
    const auto &s=input[i]; const int n=static_cast<int>(i);
    if(s.motion_type>1) throw ChainError(n,"Nieznany typ ruchu");
    for(unsigned j=0;j<6;++j) {
      if(!std::isfinite(s.target[j]) || s.target[j]<c.lower[j] || s.target[j]>c.upper[j]) throw ChainError(n,"Cel J"+std::to_string(j+1)+" poza limitami osi");
      if(!std::isfinite(s.max_velocity[j]) || s.max_velocity[j]<=0 || s.max_velocity[j]>c.max_velocity[j]*(1+1e-9) ||
         !std::isfinite(s.max_acceleration[j]) || s.max_acceleration[j]<=0 || s.max_acceleration[j]>c.max_acceleration[j]*(1+1e-9))
        throw ChainError(n,"Limity osi ruchu muszą być dodatnie i nie większe niż limity profilu");
    }
    const bool positive=std::isfinite(s.tcp.velocity) && s.tcp.velocity>0 && std::isfinite(s.tcp.acceleration) && s.tcp.acceleration>0 &&
                        std::isfinite(s.tcp.angular_velocity) && s.tcp.angular_velocity>0;
    if(s.motion_type==1 ? !positive : (s.tcp.velocity!=0 || s.tcp.acceleration!=0 || s.tcp.angular_velocity!=0))
      throw ChainError(n,"LIN wymaga dodatnich limitów TCP; PTP wymaga zerowych pól TCP");
    if(!std::isfinite(s.blend_radius) || s.blend_radius<0 || s.blend_radius>.5) throw ChainError(n,"Promień przejścia poza zakresem 0–500 mm");
  }
  robot_kinematics::Model kinematics(c.urdf,c.lower,c.upper);
  auto check=[&]() { if(cancelled && cancelled()) throw std::runtime_error("Obliczenia anulowane lub kontekst zmieniony"); };
  auto report=[&](float value,const std::string &phase) { check(); if(progress) progress(value,phase); };
  ChainResult result;

  // 1. Gęste drogi ruchów (pomijając ruchy bez przemieszczenia).
  std::vector<size_t> index;           // indeks wejścia dla każdego ruchu geometrycznego
  std::vector<SegPath> paths; size_t total_points=0;
  Joints from=c.seed;
  for(size_t i=0;i<input.size();++i) {
    const auto &s=input[i];
    if(max_abs(from,s.target)<1e-9) { result.warnings.push_back(warning(i,"no_motion","Ruch do bieżącej pozycji — pominięty")); continue; }
    SegPath path; path.start=kinematics.fk(from); path.end=kinematics.fk(s.target); path.q.push_back(from);
    if(s.motion_type==1) {
      const auto distance=(path.end.translation()-path.start.translation()).norm();
      const double rotation=Eigen::AngleAxisd(path.end.rotation()*path.start.rotation().transpose()).angle();
      const size_t steps=std::max<size_t>(1,std::ceil(std::max(distance/.001,rotation/.01)));
      if(steps>max_segment_points) throw ChainError(static_cast<int>(i),"Ruch LIN przekracza 2000 punktów IK; podziel ruch");
      auto previous=from;
      for(size_t step=1;step<=steps;++step) {
        check(); const auto solution=kinematics.solve(robot_kinematics::interpolate(path.start,path.end,double(step)/steps),previous,c.locked,1e-5,pi/1800/10,cancelled);
        if(!solution.success) throw ChainError(static_cast<int>(i),solution.message+"; punkt "+std::to_string(step));
        for(unsigned j=0;j<6;++j) if(std::abs(solution.positions[j]-previous[j])>.25) throw ChainError(static_cast<int>(i),"Przeskok gałęzi IK na drodze LIN");
        previous=solution.positions; path.q.push_back(previous);
      }
      for(unsigned j=0;j<6;++j) if(std::abs(path.q.back()[j]-s.target[j])>pi/1800)
        throw ChainError(static_cast<int>(i),"LIN prowadzi do innej konfiguracji końcowej; wybierz PTP lub zmień cel");
      path.q.back()=s.target;
    } else {
      size_t steps=std::max<size_t>(1,std::ceil(max_abs(from,s.target)/.01));
      double longest=0; Eigen::Vector3d last=path.start.translation();
      for(size_t step=1;step<=steps;++step) { const auto p=kinematics.fk(lerp(from,s.target,double(step)/steps)).translation(); longest=std::max(longest,(p-last).norm()); last=p; }
      if(longest>.001) steps*=static_cast<size_t>(std::ceil(longest/.001));
      if(steps>max_total_points) throw ChainError(static_cast<int>(i),"Ruch PTP zbyt długi do próbkowania");
      for(size_t step=1;step<=steps;++step) path.q.push_back(lerp(from,s.target,double(step)/steps));
      path.q.back()=s.target;
    }
    for(const auto &q:path.q) path.p.push_back(kinematics.fk(q).translation());
    for(size_t k=1;k<path.p.size();++k) path.length+=(path.p[k]-path.p[k-1]).norm();
    total_points+=path.q.size();
    if(total_points>max_total_points) throw ChainError(static_cast<int>(i),"Ciąg przekracza 20000 punktów drogi; podziel program");
    paths.push_back(std::move(path)); index.push_back(i); from=s.target;
    report(.3f*float(i+1)/input.size(),"Droga ruchów ciągu");
  }
  if(paths.empty()) throw ChainError(0,"Ciąg nie zawiera przemieszczenia");

  // 2. Strefy przejścia w narożnikach.
  const size_t n=paths.size();
  std::vector<Corner> corners(n);
  for(size_t i=0;i+1<n;++i) {
    const auto &segment=input[index[i]]; auto &corner=corners[i]; const auto &A=paths[i],&B=paths[i+1];
    corner.point=A.p.back(); corner.sigma_exit=double(A.q.size()-1); corner.sigma_entry=0;
    if(segment.blend_radius<=0) continue;
    double r=std::min({segment.blend_radius,.5*A.length,.5*B.length});
    const double entry_prev=i>0 && corners[i-1].blend ? corners[i-1].sigma_entry : 0.;
    std::string reason;
    for(int attempt=0;attempt<5 && r>=min_blend;++attempt,r*=.7) {
      check();
      // Wyjście: wstecz od końca A pierwsze przecięcie sfery |p−P|=r; wejście: od początku B.
      auto distance=[&](const SegPath &s,double sigma) { return (kinematics.fk(s.at(sigma)).translation()-corner.point).norm(); };
      auto crossing=[&](const SegPath &s,bool backwards)->double {
        const long last=static_cast<long>(s.q.size())-1;
        for(long k=backwards ? last-1 : 1; backwards ? k>=0 : k<=last; k+=backwards ? -1 : 1) {
          if((s.p[k]-corner.point).norm()<r) continue;
          double inside=backwards ? k+1 : k-1, outside=k;   // bisekcja na interpolacji przegubowej
          for(int it=0;it<40;++it) { const double mid=(inside+outside)/2; (distance(s,mid)<r ? inside : outside)=mid; }
          return outside;
        }
        return -1;
      };
      const double exit=crossing(A,true),entry=crossing(B,false);
      if(exit<0 || entry<0) { reason="tor nie opuszcza promienia"; continue; }
      if(exit<=entry_prev+1e-9) { reason="strefa nachodzi na poprzednie przejście"; continue; }
      const auto q0=A.at(exit),q1=B.at(entry),t0=joint_tangent(A,exit),t1=joint_tangent(B,entry);
      if(angle(t0,t1)>max_reversal) { reason="zawrócenie drogi"; r=0; break; }
      const double lambda=std::max(norm(q0,q1),1e-4);
      const size_t count=std::max<size_t>({16,static_cast<size_t>(std::ceil(1.5*max_abs(q0,q1)/.005)),static_cast<size_t>(std::ceil(2*r/.0005))});
      std::vector<Joints> curve; bool ok=true;
      for(size_t k=1;k<count && ok;++k) {
        const double u=double(k)/count,u2=u*u,u3=u2*u,u4=u3*u,u5=u4*u;
        const double h0=1-10*u3+15*u4-6*u5,h1=u-6*u3+8*u4-3*u5,h4=-4*u3+7*u4-3*u5,h5=10*u3-15*u4+6*u5;
        Joints q; for(unsigned j=0;j<6;++j) q[j]=h0*q0[j]+h1*lambda*t0[j]+h4*lambda*t1[j]+h5*q1[j];
        for(unsigned j=0;j<6;++j) if(q[j]<c.lower[j] || q[j]>c.upper[j]) ok=false;
        if(ok && (kinematics.fk(q).translation()-corner.point).norm()>std::max(1.1*r,r+.0002)) ok=false;
        curve.push_back(q);
      }
      if(!ok) { reason="krzywa przejścia wychodzi poza promień lub limity osi"; continue; }
      corner.blend=true; corner.radius=r; corner.sigma_exit=exit; corner.sigma_entry=entry; corner.curve=std::move(curve);
      break;
    }
    if(corner.blend && corner.radius<segment.blend_radius-1e-9) {
      std::ostringstream text; text.precision(1); text<<std::fixed<<"Promień ograniczony do "<<corner.radius*1e3<<" mm (połowa krótszego odcinka / kształt toru)";
      result.warnings.push_back(warning(index[i],"blend_reduced",text.str()));
    }
    if(!corner.blend) {
      if(reason.empty()) reason="odcinek zbyt krótki na torze TCP";
      result.warnings.push_back(warning(index[i],"blend_not_possible","Przejście niemożliwe ("+reason+") — postój w punkcie"));
    }
  }
  if(input[index[n-1]].blend_radius>0)
    result.warnings.push_back(warning(index[n-1],"chain_end","Ostatni ruch ciągu — postój w punkcie"));
  // Ruchy pominięte (bez przemieszczenia) po ostatnim ruchu geometrycznym też kończą ciąg.

  // 3. Podścieżki rozdzielone postojami.
  std::vector<std::vector<Vertex>> subpaths(1);
  for(size_t i=0;i<n;++i) {
    auto &out=subpaths.back(); const auto &path=paths[i];
    const double begin=i>0 && corners[i-1].blend ? corners[i-1].sigma_entry : 0.;
    const double end=corners[i].blend ? corners[i].sigma_exit : double(path.q.size()-1);
    const Label seg{static_cast<int>(i),false},blend{static_cast<int>(i),true};
    auto push=[&](const Joints &q,Label label) {
      if(!out.empty() && norm(out.back().q,q)<1e-10) { out.back().label=label; return; }
      out.push_back({q,label});
    };
    push(path.at(begin),seg);
    for(size_t k=static_cast<size_t>(std::floor(begin))+1;double(k)<end;++k) push(path.q[k],seg);
    if(corners[i].blend) {
      push(path.at(end),blend);
      for(const auto &q:corners[i].curve) push(q,blend);
    } else {
      push(path.q.back(),seg);
      if(i+1<n) subpaths.push_back({});
    }
  }

  // 4. Czas każdej podścieżki: TOTG + limity per próbka (odcinek: jego limity; przejście: minimum obu).
  auto model=make_moveit_model(c,kinematics);
  auto caps_of=[&](const Label &label) {
    const auto &a=input[index[label.segment]];
    SampleCaps caps{a.max_velocity,a.max_acceleration,a.tcp};
    if(label.blend && static_cast<size_t>(label.segment)+1<n) {
      const auto &b=input[index[label.segment+1]];
      for(unsigned j=0;j<6;++j) { caps.v[j]=std::min(caps.v[j],b.max_velocity[j]); caps.a[j]=std::min(caps.a[j],b.max_acceleration[j]); }
      if(a.tcp.velocity>0 && b.tcp.velocity>0) caps.tcp={std::min(a.tcp.velocity,b.tcp.velocity),std::min(a.tcp.acceleration,b.tcp.acceleration),
                                                         std::min(a.tcp.angular_velocity,b.tcp.angular_velocity)};
      else if(b.tcp.velocity>0) caps.tcp=b.tcp;
    }
    return caps;
  };
  Trajectory chain; chain.joint_names={"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
  struct Piece { size_t first_segment,last_segment; double offset,duration; };
  std::vector<Piece> pieces; double offset=0; size_t first=0;
  for(size_t sp=0;sp<subpaths.size();++sp) {
    const auto &vertices=subpaths[sp];
    size_t last=first; while(last+1<n && corners[last].blend) ++last;
    std::array<double,6> vmax{},amax{};
    for(size_t i=first;i<=last;++i) for(unsigned j=0;j<6;++j) {
      vmax[j]=std::max(vmax[j],input[index[i]].max_velocity[j]); amax[j]=std::max(amax[j],input[index[i]].max_acceleration[j]);
    }
    std::vector<Joints> positions; for(const auto &v:vertices) positions.push_back(v.q);
    report(.3f+.5f*float(sp)/subpaths.size(),"Czas ruchów ciągu (TOTG, limity TCP i nadgarstka)");
    Trajectory part;
    try {
      // Mała odchyłka TOTG w przegubach: łuki w wierzchołkach gęstej łamanej (krzywa przejścia) zamiast
      // skoków kierunku prędkości; na prostych odcinkach PTP/LIN praktycznie bez wpływu (kąty ~0).
      part=totg(model,positions,vmax,amax,totg_deviation,check);
      Projector projector(vertices);
      const CapProvider caps{[&]{ projector.reset(); },[&](const Joints &q) { return caps_of(projector.label(q)); }};
      limit_path_speed(part,kinematics,vmax,amax,caps,false,check);
      // Walidacja co 5 ms: osie, TCP, prosta LIN poza strefami, strefa przejścia w promieniu.
      const double total=seconds(part.points.back().time_from_start); size_t cursor=0; projector.reset();
      for(double t=0;t<=total+1e-9;t+=.005) {
        check(); const auto s=sample(part,std::min(t,total),cursor); const auto label=projector.label(s.q); const auto cap=caps_of(label);
        const auto limits=local_velocity_limits(kinematics,s,cap.v); const int seg=static_cast<int>(index[label.segment]);
        if(tcp_excess(kinematics,s,cap.tcp)>1.+1e-5) throw ChainError(seg,"Interpolacja przekracza prędkość lub przyspieszenie TCP");
        for(unsigned j=0;j<6;++j) if(!std::isfinite(s.q[j]) || s.q[j]<c.lower[j]-1e-9 || s.q[j]>c.upper[j]+1e-9 ||
            std::abs(s.v[j])>limits[j]+1e-6 || std::abs(s.a[j])>cap.a[j]+1e-5)
          throw ChainError(seg,"Interpolacja narusza limit J"+std::to_string(j+1)+" t="+std::to_string(offset+t));
        const auto pose=kinematics.fk(s.q); const auto &path=paths[label.segment];
        if(label.blend) {
          const auto &corner=corners[label.segment];
          if((pose.translation()-corner.point).norm()>std::max(1.1*corner.radius,corner.radius+.0002)+1e-4)
            throw ChainError(seg,"Przejście wychodzi poza promień");
        } else if(input[index[label.segment]].motion_type==1 && !near_path(pose,{path.start,path.end},1e-4,pi/1800))
          throw ChainError(seg,"Droga LIN odbiega od prostej");
      }
    } catch(const ChainError &) { throw; }
    catch(const std::exception &e) { throw ChainError(static_cast<int>(index[first]),e.what()); }
    const double part_duration=seconds(part.points.back().time_from_start);
    const double begin=pieces.empty() ? 0. : offset+cycle;   // postój: jeden cykl v=0 między podścieżkami
    if(begin+part_duration>max_duration+1e-9 || chain.points.size()+part.points.size()>max_points) {
      if(pieces.empty()) throw ChainError(static_cast<int>(index[first]),"Odcinek do pierwszego postoju przekracza limit czasu ciągu ("+
                                          std::to_string(std::lround(max_duration))+" s) lub 10000 punktów; dodaj postój (promień 0)");
      break;
    }
    for(auto &point:part.points) { point.time_from_start=detail::duration(begin+seconds(point.time_from_start)); chain.points.push_back(point); }
    pieces.push_back({first,last,begin,part_duration}); offset=begin+part_duration; first=last+1;
  }

  // 5. Czasy ruchów z etykiet próbek.
  const size_t planned_geometric=pieces.back().last_segment+1;
  // Ruchy bez przemieszczenia zaraz po ostatnim ujętym ruchu też są „wykonane”.
  result.planned_segments=planned_geometric<n ? index[planned_geometric] : input.size();
  std::vector<double> start(n,-1),blend_start(n,-1),finish(n,-1);
  for(const auto &piece:pieces) {
    const auto &vertices=subpaths[&piece-&pieces[0]]; Projector projector(vertices);
    for(const auto &point:chain.points) {
      const double t=seconds(point.time_from_start);
      if(t<piece.offset-1e-9 || t>piece.offset+piece.duration+1e-9) continue;
      Joints q; std::copy(point.positions.begin(),point.positions.end(),q.begin());
      const auto label=projector.label(q); const size_t i=label.segment;
      if(start[i]<0) start[i]=t;
      if(label.blend && blend_start[i]<0) blend_start[i]=t;
    }
    start[piece.first_segment]=piece.offset;
    finish[piece.last_segment]=piece.offset+piece.duration;
  }
  const double total=seconds(chain.points.back().time_from_start);
  std::vector<ChainSegmentTimingMsg> timings;
  for(size_t i=0;i<planned_geometric;++i) {
    ChainSegmentTimingMsg timing; timing.segment=index[i]; timing.tag=input[index[i]].tag;
    timing.requested_blend_radius=input[index[i]].blend_radius; timing.applied_blend_radius=corners[i].blend ? corners[i].radius : 0.;
    timing.stop=!corners[i].blend;
    timing.start_time=std::max(0.,start[i]);
    if(corners[i].blend) {
      timing.end_time=blend_start[i]>=0 ? blend_start[i] : (i+1<n && start[i+1]>=0 ? start[i+1] : timing.start_time);
      timing.transition_end=i+1<n && start[i+1]>=0 ? start[i+1] : timing.end_time;
    } else {
      timing.end_time=finish[i]>=0 ? finish[i] : timing.start_time;
      timing.transition_end=i+1<planned_geometric ? start[i+1] : total;
    }
    timing.end_time=std::max(timing.end_time,timing.start_time); timing.transition_end=std::max(timing.transition_end,timing.end_time);
    timings.push_back(timing);
  }
  // Ruchy bez przemieszczenia: chwila zakończenia poprzedniego ruchu.
  std::vector<ChainSegmentTimingMsg> all;
  for(size_t k=0,g=0;k<result.planned_segments;++k) {
    if(g<timings.size() && timings[g].segment==k) { all.push_back(timings[g++]); continue; }
    ChainSegmentTimingMsg timing; timing.segment=k; timing.tag=input[k].tag; timing.stop=true;
    timing.requested_blend_radius=input[k].blend_radius;
    timing.start_time=timing.end_time=timing.transition_end=all.empty() ? 0. : all.back().end_time;
    all.push_back(timing);
  }

  Plan &plan=result.plan;
  plan.model_id=c.model_id; plan.configuration_revision=c.configuration_revision; plan.start_positions=c.seed; plan.locked=c.locked;
  plan.trajectory=std::move(chain); plan.simulation_only=c.simulation_only; plan.simulated_start=c.simulated_start; plan.motion_type="CHAIN";
  plan.chain_segments=std::move(all);
  std::copy(plan.trajectory.points.back().positions.begin(),plan.trajectory.points.back().positions.end(),plan.target_positions.begin());
  plan.id=robot_kinematics::model_id(c.model_id+c.configuration_revision+"chain"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  plan.execution_ready=false; plan.execution_message="Ciąg ruchów: wykonanie wymaga przygotowania wykonawcy";
  report(.85,"Kompilacja podglądu F5 100 Hz");
  compile_mks(plan,c,codes,check);
  report(1,"Ciąg ruchów gotowy");
  return result;
}
}
