#include <gtest/gtest.h>
#include <fstream>
#include <iterator>
#include <algorithm>
#include "robot_motion_planner/planner.hpp"
#include "../src/planner_detail.hpp"
#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/streaming_motion.hpp"
using namespace robot_motion_planner;
namespace {
using Joints=robot_kinematics::Joints;
Context context() {
  std::ifstream input(DEMO_URDF); Context c; c.urdf={std::istreambuf_iterator<char>(input),{}}; c.model_id=robot_kinematics::model_id(c.urdf);
  c.configuration_revision="test"; c.seed={.2,.3,-.5,.2,.8,-.1}; c.known.fill(true); c.lower.fill(-3); c.upper.fill(3);
  c.max_velocity.fill(1); c.max_acceleration.fill(2); c.simulation_only=true; return c;
}
double time_of(const trajectory_msgs::msg::JointTrajectoryPoint &point) { return point.time_from_start.sec+point.time_from_start.nanosec*1e-9; }
Joints position_at(const Plan &plan,double t) {
  const auto &points=plan.trajectory.points;
  auto right=std::upper_bound(points.begin(),points.end(),t,[](double t,const auto &p){return t<time_of(p);});
  if(right==points.end()) right=points.end()-1;
  if(right==points.begin()) ++right;
  const auto &a=*(right-1),&b=*right;
  const double dt=time_of(b)-time_of(a),u=std::clamp((t-time_of(a))/dt,0.,1.);
  const double u2=u*u,u3=u2*u,u4=u3*u,u5=u4*u;
  Joints q;
  for(unsigned j=0;j<6;++j) q[j]=(1-10*u3+15*u4-6*u5)*a.positions[j]+(10*u3-15*u4+6*u5)*b.positions[j]+
    dt*((u-6*u3+8*u4-3*u5)*a.velocities[j]+(-4*u3+7*u4-3*u5)*b.velocities[j])+
    dt*dt*((u2-3*u3+3*u4-u5)*a.accelerations[j]+(u3-2*u4+u5)*b.accelerations[j])/2;
  return q;
}
ChainSegment ptp(const Joints &target,double blend,double scale=1,const std::string &tag="") {
  ChainSegment s; s.motion_type=0; s.target=target; s.max_velocity.fill(scale); s.max_acceleration.fill(2*scale); s.blend_radius=blend; s.tag=tag; return s;
}
ChainSegment lin(const Joints &target,double blend,const TcpLimits &tcp={.05,.5,1},const std::string &tag="") {
  ChainSegment s; s.motion_type=1; s.target=target; s.max_velocity.fill(1); s.max_acceleration.fill(2); s.tcp=tcp; s.blend_radius=blend; s.tag=tag; return s;
}
ChainSegment circ(const Joints &via,const Joints &target,double blend,const TcpLimits &tcp={.03,.3,1},const std::string &tag="") {
  ChainSegment s=lin(target,blend,tcp,tag); s.motion_type=2; s.via=via; return s;
}
Joints solve(const robot_kinematics::Model &model,const Context &c,const Eigen::Vector3d &offset,const Joints &seed) {
  auto pose=model.fk(c.seed); pose.translation()+=offset;
  const auto answer=model.solve(pose,seed,c.locked,1e-7,1e-6); EXPECT_TRUE(answer.success) << answer.message; return answer.positions;
}
bool has_warning(const ChainResult &r,const std::string &code) {
  return std::any_of(r.warnings.begin(),r.warnings.end(),[&](const std::string &w){ return w.find("|"+code+"|")!=std::string::npos; });
}
// Limity osi (różnice skończone) w całym planie; caps = najwyższe limity ruchów ciągu.
void check_limits(const Plan &plan,const Context &c,double vcap,double acap) {
  const double h=1e-4,total=time_of(plan.trajectory.points.back());
  for(double t=.001;t<total-.001;t+=.005) {
    const auto q=position_at(plan,t),a=position_at(plan,t-h),b=position_at(plan,t+h);
    for(unsigned j=0;j<6;++j) {
      EXPECT_LE(std::abs((b[j]-a[j])/(2*h)),vcap+1e-5) << "J" << j+1 << " t=" << t;
      EXPECT_LE(std::abs((b[j]-2*q[j]+a[j])/(h*h)),acap+2e-3) << "J" << j+1 << " t=" << t;
      EXPECT_GE(q[j],c.lower[j]-1e-9); EXPECT_LE(q[j],c.upper[j]+1e-9);
    }
  }
}
double speed_at(const Plan &plan,double t) {
  const double h=1e-4; const auto a=position_at(plan,t-h),b=position_at(plan,t+h); double s=0;
  for(unsigned j=0;j<6;++j) s+=std::pow((b[j]-a[j])/(2*h),2); return std::sqrt(s);
}
const std::array<int64_t,6> codes=[]{ std::array<int64_t,6> c; c.fill(-1); return c; }();
}

TEST(Chain, StopsAtZeroRadiusInsideOneTrajectory) {
  auto c=context(); auto q1=c.seed; q1[0]+=.3; q1[1]+=.15; auto q2=q1; q2[2]+=.3;
  const auto r=compute_chain(c,{ptp(q1,0,1,"a"),ptp(q2,0,1,"b")},90,codes);
  EXPECT_EQ(r.plan.motion_type,"CHAIN"); EXPECT_EQ(r.planned_segments,2u); ASSERT_EQ(r.plan.chain_segments.size(),2u);
  const auto &first=r.plan.chain_segments[0],&second=r.plan.chain_segments[1];
  EXPECT_EQ(first.tag,"a"); EXPECT_TRUE(first.stop); EXPECT_EQ(first.applied_blend_radius,0);
  EXPECT_NEAR(first.transition_end,second.start_time,1e-9); EXPECT_GT(second.start_time,first.end_time-1e-9);
  // Postój: punkt dokładnie w q1 z v=0.
  const auto at=position_at(r.plan,first.end_time);
  for(unsigned j=0;j<6;++j) EXPECT_NEAR(at[j],q1[j],1e-6);
  EXPECT_LT(speed_at(r.plan,first.end_time+.005),1e-3);
  for(const auto &p:r.plan.trajectory.points) for(double v:p.velocities) EXPECT_TRUE(std::isfinite(v));
  const auto &last=r.plan.trajectory.points.back(); for(unsigned j=0;j<6;++j) { EXPECT_NEAR(last.positions[j],q2[j],1e-9); EXPECT_EQ(last.velocities[j],0); }
  double previous=-1; for(const auto &p:r.plan.trajectory.points) { EXPECT_GT(time_of(p),previous); previous=time_of(p); }
  check_limits(r.plan,c,1,2);
}

TEST(Chain, PtpBlendSkipsPointWithinRadiusWithoutStopping) {
  auto c=context(); robot_kinematics::Model model(c.urdf,c.lower,c.upper);
  auto q1=c.seed; q1[0]+=.4; auto q2=q1; q2[1]+=.3; q2[2]-=.2;
  const double radius=.02;
  const auto r=compute_chain(c,{ptp(q1,radius,1,"a"),ptp(q2,0,1,"b")},90,codes);
  ASSERT_EQ(r.plan.chain_segments.size(),2u); const auto &first=r.plan.chain_segments[0];
  EXPECT_FALSE(first.stop); EXPECT_NEAR(first.applied_blend_radius,radius,1e-9) << (r.warnings.empty() ? "" : r.warnings[0]);
  EXPECT_LT(first.end_time,first.transition_end);
  const auto corner=model.fk(q1).translation().eval(); const double total=time_of(r.plan.trajectory.points.back());
  double nearest=1e9;
  for(double t=0;t<=total;t+=.002) nearest=std::min(nearest,(model.fk(position_at(r.plan,t)).translation()-corner).norm());
  EXPECT_GT(nearest,.001);   // punkt pominięty
  for(double t=first.end_time;t<=first.transition_end;t+=.002) {
    EXPECT_LE((model.fk(position_at(r.plan,t)).translation()-corner).norm(),1.1*radius+2e-4) << t;
    EXPECT_GT(speed_at(r.plan,t),1e-3) << "postój w strefie przejścia t=" << t;
  }
  check_limits(r.plan,c,1,2);
  // Bez przejścia ten sam ciąg trwa dłużej.
  const auto stop=compute_chain(c,{ptp(q1,0),ptp(q2,0)},90,codes);
  EXPECT_LT(total,time_of(stop.plan.trajectory.points.back()));
}

TEST(Chain, LinBlendKeepsLinesOutsideZoneAndTcpLimits) {
  auto c=context(); robot_kinematics::Model model(c.urdf,c.lower,c.upper);
  const auto q1=solve(model,c,{.04,0,0},c.seed),q2=solve(model,c,{.04,.04,0},q1);
  const TcpLimits tcp{.03,.3,1}; const double radius=.01;
  const auto r=compute_chain(c,{lin(q1,radius,tcp,"a"),lin(q2,0,tcp,"b")},90,codes);
  ASSERT_EQ(r.plan.chain_segments.size(),2u); const auto &first=r.plan.chain_segments[0],&second=r.plan.chain_segments[1];
  EXPECT_NEAR(first.applied_blend_radius,radius,1e-9);
  EXPECT_NEAR(first.transition_end,second.start_time,1e-9); EXPECT_NEAR(second.transition_end,time_of(r.plan.trajectory.points.back()),1e-9);
  const auto p0=model.fk(c.seed).translation().eval(),p1=model.fk(q1).translation().eval(),p2=model.fk(q2).translation().eval();
  auto line_distance=[](const Eigen::Vector3d &p,const Eigen::Vector3d &a,const Eigen::Vector3d &b) {
    const auto d=(b-a).eval(); const double t=std::clamp((p-a).dot(d)/d.squaredNorm(),0.,1.); return (a+t*d-p).norm(); };
  const double total=time_of(r.plan.trajectory.points.back()),h=1e-4;
  for(double t=.001;t<total-.001;t+=.005) {
    const auto p=model.fk(position_at(r.plan,t)).translation().eval();
    if(t<first.end_time-.011) EXPECT_LE(line_distance(p,p0,p1),1.2e-4) << t;
    else if(t>first.transition_end+.011) EXPECT_LE(line_distance(p,p1,p2),1.2e-4) << t;
    else EXPECT_LE((p-p1).norm(),1.1*radius+3e-4) << t;
    const auto a=model.fk(position_at(r.plan,t-h)).translation().eval(),b=model.fk(position_at(r.plan,t+h)).translation().eval();
    EXPECT_LE((b-a).norm()/(2*h),tcp.velocity+1e-5) << t;
    EXPECT_LE((b-2*p+a).norm()/(h*h),tcp.acceleration+2e-3) << t;
  }
  check_limits(r.plan,c,1,2);
}

TEST(Chain, CircFollowsArcThroughViaWithTcpLimits) {
  auto c=context(); robot_kinematics::Model model(c.urdf,c.lower,c.upper);
  const auto via=solve(model,c,{.03,.03,0},c.seed),target=solve(model,c,{.06,0,0},c.seed);
  const TcpLimits tcp{.03,.3,1};
  const auto r=compute_chain(c,{circ(via,target,0,tcp,"c")},90,codes);
  ASSERT_EQ(r.plan.chain_segments.size(),1u); EXPECT_EQ(r.plan.chain_segments[0].tag,"c");
  const auto p0=model.fk(c.seed).translation().eval(),center=(p0+Eigen::Vector3d(.03,0,0)).eval();
  const double total=time_of(r.plan.trajectory.points.back()),h=1e-4; double top=-1;
  for(double t=0;t<=total;t+=.005) {
    const auto p=model.fk(position_at(r.plan,t)).translation().eval();
    EXPECT_NEAR((p-center).norm(),.03,1.2e-4) << t; EXPECT_NEAR(p.z(),p0.z(),1.2e-4) << t;
    EXPECT_GE(p.y()-p0.y(),-1.2e-4) << t;   // półokrąg po stronie punktu pośredniego
    top=std::max(top,p.y()-p0.y());
    if(t>.001 && t<total-.001) {
      const auto a=model.fk(position_at(r.plan,t-h)).translation().eval(),b=model.fk(position_at(r.plan,t+h)).translation().eval();
      EXPECT_LE((b-a).norm()/(2*h),tcp.velocity+1e-5) << t;
    }
  }
  EXPECT_NEAR(top,.03,2e-4);
  EXPECT_GE(total,M_PI*.03/tcp.velocity);
  for(unsigned j=0;j<6;++j) EXPECT_NEAR(r.plan.target_positions[j],target[j],1e-9);
  check_limits(r.plan,c,1,2);
}

TEST(Chain, CircPathOrientationTurnsWithArcAroundCircleAxis) {
  auto c=context(); robot_kinematics::Model model(c.urdf,c.lower,c.upper);
  const auto start=model.fk(c.seed); const Eigen::Vector3d p0=start.translation(),center=p0+Eigen::Vector3d(.02,0,0);
  auto pose_at=[&](double deg,double turn) { auto p=start; const double a=deg*M_PI/180;
    p.translation()=center+.02*Eigen::Vector3d(std::cos(a),std::sin(a),0); p.linear()=Eigen::AngleAxisd(turn*M_PI/180,Eigen::Vector3d::UnitZ())*start.rotation(); return p; };
  auto solve_pose=[&](const robot_kinematics::Pose &p) { const auto r=model.solve(p,c.seed,c.locked,1e-7,1e-6); EXPECT_TRUE(r.success) << r.message; return r.positions; };
  // Łuk 45° od 180° do 225° (przeciwnie do wskazówek wokół +Z): cel obrócony o 45° wokół osi okręgu.
  const auto via=solve_pose(pose_at(202.5,0)),target=solve_pose(pose_at(225,45));
  auto segment=circ(via,target,0,{.02,.2,1},"p"); segment.circ_orientation=1;
  const auto r=compute_chain(c,{segment},90,codes);
  const double total=time_of(r.plan.trajectory.points.back());
  for(double t=0;t<=total;t+=.01) {
    const auto pose=model.fk(position_at(r.plan,t)); const Eigen::Vector3d d=pose.translation()-center;
    EXPECT_NEAR(d.head<2>().norm(),.02,1.2e-4) << t;
    double angle=std::atan2(d.y(),d.x())*180/M_PI; if(angle<0) angle+=360;   // 180…225°
    const Eigen::AngleAxisd turn(pose.rotation()*start.rotation().transpose());
    EXPECT_NEAR(std::abs(turn.angle()*turn.axis().z())*180/M_PI,angle-180,.2) << t;   // obrót wokół Z = kąt łuku
  }
  // Limit przyspieszenia kątowego TCP (tcp.angular_acceleration): ruch dłuższy, |dω/dt| w limicie.
  auto limited=segment; limited.tcp.angular_acceleration=.5;
  const auto slow=compute_chain(c,{limited},90,codes); const double slow_total=time_of(slow.plan.trajectory.points.back());
  EXPECT_GT(slow_total,total*1.05);
  auto omega=[&](double t) { const double h=1e-3;
    const Eigen::AngleAxisd d(model.fk(position_at(slow.plan,t+h)).rotation()*model.fk(position_at(slow.plan,t-h)).rotation().transpose());
    return (d.axis()*d.angle()/(2*h)).eval(); };
  double peak=0;
  for(double t=.01;t<slow_total-.01;t+=.005) peak=std::max(peak,((omega(t+.002)-omega(t-.002))/.004).norm());
  EXPECT_LE(peak,.5*1.05) << "przyspieszenie kątowe TCP";
  limited.tcp.angular_acceleration=-1; EXPECT_THROW(compute_chain(c,{limited},90,codes),ChainError);
  segment.circ_orientation=2; EXPECT_THROW(compute_chain(c,{segment},90,codes),ChainError);
}

TEST(Chain, CircBlendsWithLin) {
  auto c=context(); robot_kinematics::Model model(c.urdf,c.lower,c.upper);
  const auto q1=solve(model,c,{.02,0,0},c.seed),via=solve(model,c,{.035,.015,0},q1),q2=solve(model,c,{.05,0,0},q1),q3=solve(model,c,{.05,-.03,0},q2);
  const auto r=compute_chain(c,{lin(q1,.005,{.03,.3,1},"a"),circ(via,q2,.005,{.03,.3,1},"b"),lin(q3,0,{.03,.3,1},"c")},90,codes);
  ASSERT_EQ(r.plan.chain_segments.size(),3u);
  EXPECT_FALSE(r.plan.chain_segments[0].stop); EXPECT_FALSE(r.plan.chain_segments[1].stop);
  EXPECT_NEAR(r.plan.chain_segments[0].applied_blend_radius,.005,1e-9); EXPECT_NEAR(r.plan.chain_segments[1].applied_blend_radius,.005,1e-9);
  check_limits(r.plan,c,1,2);
}

TEST(Chain, CircRejectsDegenerateGeometry) {
  auto c=context(); robot_kinematics::Model model(c.urdf,c.lower,c.upper);
  const auto mid=solve(model,c,{.02,0,0},c.seed),end=solve(model,c,{.04,0,0},c.seed),off=solve(model,c,{.02,.02,0},c.seed);
  try { compute_chain(c,{ptp(off,0),circ(mid,end,0)},90,codes); } catch(const ChainError &) { FAIL() << "ruch poprawny"; }
  try { compute_chain(c,{circ(mid,end,0)},90,codes); FAIL(); } catch(const ChainError &e) { EXPECT_EQ(e.segment,0); EXPECT_NE(std::string(e.what()).find("prostej"),std::string::npos); }
  try { compute_chain(c,{ptp(off,0),circ(mid,off,0)},90,codes); FAIL(); } catch(const ChainError &e) { EXPECT_EQ(e.segment,1); }
  try { compute_chain(c,{circ(off,end,0,{0,0,0})},90,codes); FAIL(); } catch(const ChainError &e) { EXPECT_EQ(e.segment,0); }
  auto outside=off; outside[0]=4;
  try { compute_chain(c,{circ(outside,end,0)},90,codes); FAIL(); } catch(const ChainError &e) { EXPECT_NE(std::string(e.what()).find("pośredni"),std::string::npos); }
}

TEST(Chain, RadiusIsLimitedToHalfOfShorterSegment) {
  auto c=context(); robot_kinematics::Model model(c.urdf,c.lower,c.upper);
  const auto q1=solve(model,c,{.03,0,0},c.seed),q2=solve(model,c,{.03,.03,0},q1);
  const auto r=compute_chain(c,{lin(q1,.1),lin(q2,0)},90,codes);
  EXPECT_TRUE(has_warning(r,"blend_reduced"));
  EXPECT_LE(r.plan.chain_segments[0].applied_blend_radius,.015+1e-9); EXPECT_GT(r.plan.chain_segments[0].applied_blend_radius,.005);
}

TEST(Chain, ReversalAndPureWristRotationFallBackToStop) {
  auto c=context(); auto q1=c.seed; q1[0]+=.3;
  const auto back=compute_chain(c,{ptp(q1,.02),ptp(c.seed,0)},90,codes);
  EXPECT_TRUE(has_warning(back,"blend_not_possible")); EXPECT_TRUE(back.plan.chain_segments[0].stop);
  auto wrist=q1; wrist[5]+=.3;   // sam obrót narzędzia: tor TCP ~0
  const auto turn=compute_chain(c,{ptp(q1,.02),ptp(wrist,0)},90,codes);
  EXPECT_TRUE(has_warning(turn,"blend_not_possible")); EXPECT_TRUE(turn.plan.chain_segments[0].stop);
  const auto end=compute_chain(c,{ptp(q1,.02)},90,codes);
  EXPECT_TRUE(has_warning(end,"chain_end"));
}

TEST(Chain, DifferentSpeedsKeepOwnLimits) {
  auto c=context(); auto q1=c.seed; q1[0]+=.4; auto q2=q1; q2[1]+=.4;
  const auto r=compute_chain(c,{ptp(q1,0,.2,"slow"),ptp(q2,0,1,"fast")},90,codes);
  const auto &slow=r.plan.chain_segments[0];
  const double h=1e-4;
  for(double t=.001;t<slow.end_time;t+=.005) {
    const auto a=position_at(r.plan,t-h),b=position_at(r.plan,t+h);
    for(unsigned j=0;j<6;++j) EXPECT_LE(std::abs(b[j]-a[j])/(2*h),.2+1e-5) << t;
  }
  double peak=0; for(double t=slow.transition_end;t<time_of(r.plan.trajectory.points.back());t+=.005) peak=std::max(peak,speed_at(r.plan,t));
  EXPECT_GT(peak,.5);
}

TEST(Chain, TruncatesAtStopWhenTooLongAndSkipsZeroMoves) {
  auto c=context(); auto q1=c.seed; q1[0]+=.5; auto q2=q1; q2[1]+=.5; auto q3=q2; q3[0]-=.5;
  const auto full=compute_chain(c,{ptp(q1,0),ptp(q2,0),ptp(q3,0)},90,codes);
  const double first=full.plan.chain_segments[0].end_time;
  const auto cut=compute_chain(c,{ptp(q1,0),ptp(q2,0),ptp(q3,0)},first+.5,codes);
  EXPECT_EQ(cut.planned_segments,1u); EXPECT_EQ(cut.plan.chain_segments.size(),1u);
  for(unsigned j=0;j<6;++j) EXPECT_NEAR(cut.plan.trajectory.points.back().positions[j],q1[j],1e-9);
  EXPECT_THROW(compute_chain(c,{ptp(q1,0)},first*.5,codes),ChainError);
  const auto same=compute_chain(c,{ptp(q1,0,1,"a"),ptp(q1,0,1,"b"),ptp(q2,0,1,"c")},90,codes);
  EXPECT_TRUE(has_warning(same,"no_motion")); ASSERT_EQ(same.plan.chain_segments.size(),3u); EXPECT_EQ(same.plan.chain_segments[1].tag,"b");
  try { compute_chain(c,{ptp(q1,0),lin(q2,0,{0,0,0})},90,codes); FAIL(); } catch(const ChainError &e) { EXPECT_EQ(e.segment,1); }
}

TEST(Chain, PhysicalRequiresProfilesAndCompilesMks) {
  auto c=context(); c.simulation_only=false; auto q1=c.seed; q1[0]+=.2;
  EXPECT_THROW(compute_chain(c,{ptp(q1,0)},90,codes),std::invalid_argument);
  for(unsigned i=0;i<6;++i) { auto &p=c.profiles[i]; p.configured=true; p.gear_ratio=1; p.direction=1; p.lower_rad=-3; p.upper_rad=3; p.velocity_rad_s=1; p.acceleration_rad_s2=2; }
  std::array<int64_t,6> acc; acc.fill(135);
  const auto r=compute_chain(c,{ptp(q1,0)},90,acc);
  EXPECT_TRUE(r.plan.mks_compatible) << r.plan.mks_message; EXPECT_FALSE(r.plan.simulation_only);
  int calls=0; EXPECT_THROW(compute_chain(c,{ptp(q1,0)},90,acc,[&]{ return ++calls>20; }),std::runtime_error);
}

// Regresja 10.11.2026: PTP ≥ ~50% odrzucany przez wykonawcę („Trajektoria streaming przekracza limity osi”) — otoczka
// Bernsteina na skoku przyspieszenia TOTG. Po podziale odcinków tego samego wielomianu wykonawca przyjmuje plan.
TEST(Chain,RefineForStreamingSatisfiesExecutorEnvelope) {
  Context c=context(); const double vmax=80*M_PI/180,amax=500*M_PI/180;
  for(unsigned j=0;j<6;++j) { auto &p=c.profiles[j]; p.axis=j+1; p.configured=true; p.gear_ratio=10; p.direction=1; p.lower_rad=-3; p.upper_rad=3;
    p.velocity_rad_s=vmax; p.acceleration_rad_s2=amax; }
  // Rozpędzanie i hamowanie (bang-bang, 60% przyspieszenia; rzeczywiste maksimum wielomianu ≈ 82% limitu) co h; w węzłach postoju i przełączenia a=0 — jak w wyniku TOTG.
  detail::Trajectory t; double h=.0123,a=.6*amax;
  auto add=[&](double time,double q,double v,double acc) { trajectory_msgs::msg::JointTrajectoryPoint p; p.positions.assign(6,0); p.velocities.assign(6,0);
    p.accelerations.assign(6,0); p.positions[4]=q; p.velocities[4]=v; p.accelerations[4]=acc; p.time_from_start=detail::duration(time); t.points.push_back(p); };
  add(0,0,0,0); add(h,a*h*h/2,a*h,a); add(2*h,2*a*h*h,2*a*h,0); add(3*h,3.5*a*h*h,a*h,-a); add(4*h,4*a*h*h,0,0);
  std::array<mks_servo_can::AxisConfig,6> axes; std::array<std::uint16_t,6> micro; micro.fill(128);
  for(unsigned j=0;j<6;++j) { axes[j]={static_cast<std::uint16_t>(j+1),10,1,0,-3,3,vmax,amax,0,250,{}}; axes[j].streaming.acceleration=250; }
  auto executor=[&](const detail::Trajectory &tr) {
    mks_servo_can::PreparedMotionProgram p; p.participating.fill(true);
    for(const auto &point:tr.points) { mks_servo_can::ProgramPoint q; q.seconds=time_of(point);
      for(unsigned j=0;j<6;++j) { q.positions[j]=point.positions[j]; q.velocities[j]=point.velocities[j]; q.accelerations[j]=point.accelerations[j]; } p.points.push_back(q); }
    return mks_servo_can::compile_streaming_program(p,axes,micro);
  };
  EXPECT_THROW(executor(t),std::invalid_argument);
  const auto before=t; const auto added=detail::refine_for_streaming(t,c);
  EXPECT_GT(added,0u); EXPECT_LE(added,60u);
  EXPECT_NO_THROW(executor(t));
  // Węzły pierwotne zostają, punkty pośrednie leżą na tym samym wielomianie (sample()).
  size_t cursor=0; for(const auto &point:t.points) { const auto s=detail::sample(before,time_of(point),cursor);
    EXPECT_NEAR(point.positions[4],s.q[4],1e-12); EXPECT_NEAR(point.velocities[4],s.v[4],1e-9); }
  // Wielomian bez zmian: punkty pośrednie leżą na pierwotnej krzywej.
  EXPECT_NEAR(time_of(t.points.back()),4*h,1e-9);
  // Trajektoria naprawdę za szybka (rzeczywiste maksimum ≈ 124% limitu) — podział nie ukrywa przekroczenia.
  t.points.clear(); a=.9*amax;
  add(0,0,0,0); add(h,a*h*h/2,a*h,a); add(2*h,2*a*h*h,2*a*h,0); add(3*h,3.5*a*h*h,a*h,-a); add(4*h,4*a*h*h,0,0);
  detail::refine_for_streaming(t,c); EXPECT_THROW(executor(t),std::invalid_argument);
  // Bez pełnych profili — bez zmian.
  Context bare=context(); auto copy=t; EXPECT_EQ(detail::refine_for_streaming(copy,bare),0u); EXPECT_EQ(copy.points.size(),t.points.size());
}

TEST(Chain, PtpSegmentIgnoresWristSingularityLinStillSlows) {
  // J5=0 (osobliwość nadgarstka): PTP w ciągu z pełną prędkością J4, LIN (tor TCP) — J4/J6 ≤ 5°/s w strefie.
  auto c=context(); c.seed[4]=0; auto q1=c.seed; q1[3]+=.4;
  const auto r=compute_chain(c,{ptp(q1,0)},90,codes);
  double fast=0; for(const auto &p:r.plan.trajectory.points) fast=std::max(fast,std::abs(p.velocities[3]));
  EXPECT_GT(fast,10*3.141592653589793/180);
  auto q2=c.seed; q2[4]=.05; q2[5]+=.3; c.seed[4]=.05;   // obrót narzędzia przy J5≈2,9°
  const auto l=compute_chain(c,{lin(q2,0)},90,codes);
  double wrist=0; for(const auto &p:l.plan.trajectory.points) wrist=std::max(wrist,std::abs(p.velocities[5]));
  EXPECT_LE(wrist,5*3.141592653589793/180+1e-6);
}
