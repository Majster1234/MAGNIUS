#include <gtest/gtest.h>
#include <fstream>
#include <iterator>
#include <algorithm>
#include "robot_motion_planner/planner.hpp"
using namespace robot_motion_planner;
namespace {
Context context() {
  std::ifstream input(DEMO_URDF); Context c;c.urdf={std::istreambuf_iterator<char>(input),{}};c.model_id=robot_kinematics::model_id(c.urdf); c.configuration_revision="test";
  c.seed={.2,.3,-.5,.2,.2,-.1};c.known.fill(true);c.lower.fill(-3);c.upper.fill(3);c.max_velocity.fill(1);c.max_acceleration.fill(2); return c;
}
constexpr double rad=3.141592653589793/180;
double time_of(const trajectory_msgs::msg::JointTrajectoryPoint &point) { return point.time_from_start.sec+point.time_from_start.nanosec*1e-9; }
// Niezależna postać bazowa Hermite'a: sprawdzamy interpolację q/v/a stosowaną przez JTC,
// a prędkości/przyspieszenia oceniamy przez różnice skończone tej pozycji.
robot_kinematics::Joints position_at(const Plan &plan,double t) {
  const auto &points=plan.trajectory.points;
  auto right=std::upper_bound(points.begin(),points.end(),t,[](double t,const auto &p){return t<time_of(p);});
  if(right==points.end()) right=points.end()-1;
  if(right==points.begin()) ++right;
  const auto &a=*(right-1),&b=*right;
  const double dt=time_of(b)-time_of(a),u=std::clamp((t-time_of(a))/dt,0.,1.);
  const double u2=u*u,u3=u2*u,u4=u3*u,u5=u4*u;
  robot_kinematics::Joints q;
  for(unsigned j=0;j<6;++j) q[j]=(1-10*u3+15*u4-6*u5)*a.positions[j]+(10*u3-15*u4+6*u5)*b.positions[j]+
    dt*((u-6*u3+8*u4-3*u5)*a.velocities[j]+(-4*u3+7*u4-3*u5)*b.velocities[j])+
    dt*dt*((u2-3*u3+3*u4-u5)*a.accelerations[j]+(u3-2*u4+u5)*b.accelerations[j])/2;
  return q;
}
void check_interpolation(const Plan &plan,const Context &c,const robot_kinematics::Model &model) {
  const double h=1e-4, total=time_of(plan.trajectory.points.back());
  for(double t=.001;t<total-.001;t+=.005) {
    const auto q=position_at(plan,t),a=position_at(plan,t-h),b=position_at(plan,t+h);
    const double blend=wrist_singularity_blend(model,q);
    for(unsigned j=0;j<6;++j) {
      const double limit=(j==3 || j==5) && c.max_velocity[j]>5*rad ? 5*rad+(c.max_velocity[j]-5*rad)*blend : c.max_velocity[j];
      EXPECT_LE(std::abs((b[j]-a[j])/(2*h)),limit+1e-5) << "J" << j+1 << " t=" << t;
      EXPECT_LE(std::abs((b[j]-2*q[j]+a[j])/(h*h)),c.max_acceleration[j]+1e-3) << "J" << j+1 << " t=" << t;
      EXPECT_GE(q[j],c.lower[j]-1e-9); EXPECT_LE(q[j],c.upper[j]+1e-9);
    }
  }
}
}
TEST(Planner, CompletePathAndNoHardwareReadiness) {
  auto c=context();robot_kinematics::Model model(c.urdf,c.lower,c.upper);
  auto q=c.seed;q[0]+=.01;q[1]+=.005; auto goal=model.fk(q);
  std::array<int64_t,6> codes;codes.fill(-1);
  auto plan=compute(c,{goal},.002,.01,.5,.5,codes);
  ASSERT_GE(plan.trajectory.points.size(),2u);EXPECT_FALSE(plan.execution_ready);EXPECT_FALSE(plan.mks_compatible);
  EXPECT_EQ(plan.model_id,c.model_id);EXPECT_EQ(plan.start_positions,c.seed);EXPECT_EQ(plan.trajectory.joint_names[5],"joint_6");
  const auto &last=plan.trajectory.points.back();robot_kinematics::Joints end;
  std::copy(last.positions.begin(),last.positions.end(),end.begin());
  EXPECT_LT((model.fk(end).translation()-goal.translation()).norm(),.0001);
  double previous=-1;for(const auto &point:plan.trajectory.points) { double t=point.time_from_start.sec+point.time_from_start.nanosec*1e-9;EXPECT_GT(t,previous);previous=t; }
}
TEST(Planner, FrozenAxesUnknownPositionAndCancellation) {
  auto c=context(); c.locked.fill(true);robot_kinematics::Model model(c.urdf,c.lower,c.upper);
  std::array<int64_t,6> codes;codes.fill(-1);
  EXPECT_THROW(compute(c,{model.fk(c.seed)},.001,.01,1,1,codes,[](){return true;}),std::runtime_error);
  c.known[2]=false;EXPECT_THROW(validate_context(c),std::invalid_argument);
  c.known[2]=true;c.model_id="stale";EXPECT_THROW(validate_context(c),std::invalid_argument);
}
TEST(Planner, CombinedCornersFrozenWristAndMksHoldPreview) {
  auto c=context(); c.locked[5]=true; robot_kinematics::Model model(c.urdf,c.lower,c.upper);
  auto q=c.seed; q[0]+=.01; auto corner=model.fk(q); q[1]+=.005;
  std::array<int64_t,6> codes; codes.fill(2);
  const auto path=compute(c,{corner,model.fk(q)},.002,.01,.5,.5,codes);
  for(const auto &point:path.trajectory.points) EXPECT_NEAR(point.positions[5],c.seed[5],1e-9);
  for(unsigned i=0;i<6;++i) {
    auto &p=c.profiles[i]; p.configured=true; p.gear_ratio=1; p.direction=1; p.lower_rad=-3; p.upper_rad=3;
    p.velocity_rad_s=1; p.acceleration_rad_s2=2;
  }
  const auto hold=compute(c,{model.fk(c.seed)},.002,.01,1,1,codes);
  ASSERT_TRUE(hold.mks_compatible) << hold.mks_message; ASSERT_GE(hold.mks_samples.size(),2u);
  for(const auto &frame:hold.mks_samples) for(unsigned i=0;i<6;++i) {
    EXPECT_EQ(frame.operation[i],0); EXPECT_EQ(frame.speed_rpm[i],1); EXPECT_EQ(frame.acceleration_code[i],2);
    EXPECT_EQ(frame.payloads[i*8],0xf5);
  }
  EXPECT_FALSE(hold.execution_ready);
}
TEST(PreviewMotion, PtpSynchronizesSixAxesAndPreservesTurns) {
  auto c=context(); c.simulation_only=true; c.lower.fill(-20); c.upper.fill(20); c.seed[0]+=6.283185307179586;
  auto target=c.seed; for(unsigned i=0;i<6;++i) target[i]+=.1*(i+1);
  std::array<int64_t,6> codes; codes.fill(-1);
  const auto plan=compute_preview(c,target,0,codes);
  EXPECT_TRUE(plan.simulation_only); EXPECT_FALSE(plan.execution_ready); EXPECT_EQ(plan.motion_type,"PTP");
  for(unsigned j=0;j<6;++j) {
    EXPECT_NEAR(plan.trajectory.points.front().positions[j],c.seed[j],1e-9);
    EXPECT_NEAR(plan.trajectory.points.back().positions[j],target[j],1e-9);
    for(const auto &point:plan.trajectory.points) {
      EXPECT_LE(std::abs(point.velocities[j]),c.max_velocity[j]+1e-6);
      EXPECT_LE(std::abs(point.accelerations[j]),c.max_acceleration[j]+1e-5);
      EXPECT_NEAR((point.positions[j]-c.seed[j])/(target[j]-c.seed[j]),(point.positions[0]-c.seed[0])/(target[0]-c.seed[0]),1e-7);
    }
  }
  EXPECT_THROW(compute_preview(c,target,0,codes,[](){return true;}),std::runtime_error);
}
TEST(PreviewMotion, LinKeepsTcpOnLineAndRejectsDifferentFinalBranch) {
  auto c=context(); c.simulation_only=true; robot_kinematics::Model model(c.urdf,c.lower,c.upper);
  auto target=c.seed; target[0]+=.005; target[1]+=.003;
  std::array<int64_t,6> codes; codes.fill(-1);
  const auto plan=compute_preview(c,target,1,codes);
  const auto a=model.fk(c.seed),b=model.fk(target); const auto d=(b.translation()-a.translation()).eval();
  for(const auto &point:plan.trajectory.points) {
    robot_kinematics::Joints q; std::copy(point.positions.begin(),point.positions.end(),q.begin()); auto pose=model.fk(q);
    const double t=std::clamp((pose.translation()-a.translation()).dot(d)/d.squaredNorm(),0.,1.);
    auto expected=robot_kinematics::interpolate(a,b,t);
    EXPECT_LE((pose.translation()-expected.translation()).norm(),1e-4);
    EXPECT_LE(Eigen::AngleAxisd(pose.rotation()*expected.rotation().transpose()).angle(),3.141592653589793/1800);
  }
  c.lower.fill(-20); c.upper.fill(20); target=c.seed; target[5]+=6.283185307179586;
  EXPECT_THROW(compute_preview(c,target,1,codes),std::runtime_error);
  target[0]=30; EXPECT_THROW(compute_preview(c,target,0,codes),std::invalid_argument);
}
TEST(PreviewMotion, PureRotationIgnoresSubToleranceIkTranslationForProgress) {
  auto c=context();c.simulation_only=true;c.seed={15*rad,20*rad,-35*rad,20*rad,45*rad,-10*rad};
  robot_kinematics::Model model(c.urdf,c.lower,c.upper);const auto start=model.fk(c.seed);
  std::array<int64_t,6> codes;codes.fill(-1);
  for(double angle : {.1,1.,5.,10.,20.}) {
    auto goal=start;goal.linear()=Eigen::AngleAxisd(angle*rad,Eigen::Vector3d::UnitZ()).toRotationMatrix()*start.rotation();
    const auto target=model.solve(goal,c.seed,c.locked,1e-6,1e-5);ASSERT_TRUE(target.success)<<target.message;
    const auto plan=compute_preview(c,target.positions,1,codes,{}, {},TcpLimits{.01,.1,10*rad});
    EXPECT_FALSE(plan.execution_ready);
    for(double t=0;t<=time_of(plan.trajectory.points.back());t+=.005) {
      const auto pose=model.fk(position_at(plan,t));
      EXPECT_LE((pose.translation()-start.translation()).norm(),1e-4);
      const Eigen::AngleAxisd rotation(pose.rotation()*start.rotation().transpose());
      const double u=std::clamp(rotation.axis().z()*rotation.angle()/(angle*rad),0.,1.);
      const auto expected=robot_kinematics::interpolate(start,goal,u);
      EXPECT_LE(Eigen::AngleAxisd(pose.rotation()*expected.rotation().transpose()).angle(),.1*rad);
    }
  }
}
TEST(WristSpeed, SmoothBandUsesActualAngularJacobian) {
  auto c=context(); robot_kinematics::Model model(c.urdf,c.lower,c.upper); auto q=c.seed;
  double previous=-1;
  for(double angle=0;angle<=40;angle+=.5) {
    q[4]=angle*rad; const double blend=wrist_singularity_blend(model,q);
    EXPECT_GE(blend,previous-1e-10); EXPECT_GE(blend,0); EXPECT_LE(blend,1); previous=blend;
    if(angle<=5) EXPECT_NEAR(blend,0,1e-12);
    if(angle>=30) EXPECT_NEAR(blend,1,1e-12);
    q[4]=-angle*rad; EXPECT_NEAR(wrist_singularity_blend(model,q),blend,1e-10);
  }
}
TEST(WristSpeed, BothAxesCappedAndFasterAwayWithExactPtpEndpoints) {
  auto c=context(); c.simulation_only=true;c.seed[4]=0;
  auto target=c.seed;target[3]+=.4;target[5]-=.3;target[0]+=.02;
  std::array<int64_t,6> codes; codes.fill(-1);
  auto near=compute_preview(c,target,0,codes); robot_kinematics::Model model(c.urdf,c.lower,c.upper);
  EXPECT_FALSE(near.execution_ready); EXPECT_GE(time_of(near.trajectory.points.back()),.4/(5*rad));
  for(unsigned j=0;j<6;++j) {
    EXPECT_NEAR(near.trajectory.points.front().positions[j],c.seed[j],1e-9);
    EXPECT_NEAR(near.trajectory.points.back().positions[j],target[j],1e-9);
  }
  check_interpolation(near,c,model);
  c.seed[4]=target[4]=1.; const auto far=compute_preview(c,target,0,codes);
  double fast=0; for(const auto &p:far.trajectory.points) fast=std::max(fast,std::abs(p.velocities[3]));
  EXPECT_GT(fast,5*rad*2);EXPECT_LT(time_of(far.trajectory.points.back()),time_of(near.trajectory.points.back())*.7);
  check_interpolation(far,c,model);
}
TEST(WristSpeed, BrakesBeforeZoneAndAcceleratesAfterLeavingIt) {
  auto c=context();c.simulation_only=true;c.seed[4]=.65;
  auto target=c.seed;target[3]+=1.2;target[5]-=.9;target[4]=-.65;
  std::array<int64_t,6> codes;codes.fill(-1);
  const auto plan=compute_preview(c,target,0,codes);robot_kinematics::Model model(c.urdf,c.lower,c.upper);
  check_interpolation(plan,c,model);
  double near=0,far=0;
  for(const auto &p:plan.trajectory.points) {
    if(std::abs(p.positions[4])<5*rad) near=std::max(near,std::abs(p.velocities[3]));
    if(std::abs(p.positions[4])>30*rad) far=std::max(far,std::abs(p.velocities[3]));
  }
  EXPECT_LE(near,5*rad+1e-6);EXPECT_GT(near,rad);EXPECT_GT(far,5*rad);
}
TEST(WristSpeed, DoesNotRaiseLowerCommandsOrSlowAStationaryWrist) {
  auto c=context();c.simulation_only=true;c.seed[4]=0;
  std::array<int64_t,6> codes;codes.fill(-1);
  auto target=c.seed;target[0]+=.4;
  const auto near=compute_preview(c,target,0,codes);
  c.seed[4]=target[4]=1.;const auto far=compute_preview(c,target,0,codes);
  EXPECT_NEAR(time_of(near.trajectory.points.back()),time_of(far.trajectory.points.back()),1e-9);
  c.seed[4]=target[4]=0;c.max_velocity.fill(3*rad);target=c.seed;target[3]+=.05;target[5]+=.03;
  const auto slow=compute_preview(c,target,0,codes);robot_kinematics::Model model(c.urdf,c.lower,c.upper);
  check_interpolation(slow,c,model);
}
TEST(WristSpeed, LinKeepsFullTcpPoseAfterChangingTime) {
  auto c=context();c.simulation_only=true;c.seed[4]=.05;
  auto target=c.seed;target[5]+=.3;
  std::array<int64_t,6> codes;codes.fill(-1);
  const auto plan=compute_preview(c,target,1,codes);robot_kinematics::Model model(c.urdf,c.lower,c.upper);
  check_interpolation(plan,c,model);
  const auto start=model.fk(c.seed),end=model.fk(target);
  for(double t=0;t<time_of(plan.trajectory.points.back());t+=.01) {
    const auto pose=model.fk(position_at(plan,t));
    EXPECT_LE((pose.translation()-start.translation()).norm(),1e-4);
    const Eigen::AngleAxisd full(end.rotation()*start.rotation().transpose()),actual(pose.rotation()*start.rotation().transpose());
    const double u=std::clamp(actual.axis().dot(full.axis())*actual.angle()/full.angle(),0.,1.);
    const auto expected=robot_kinematics::interpolate(start,end,u);
    EXPECT_LE(Eigen::AngleAxisd(pose.rotation()*expected.rotation().transpose()).angle(),rad*.1);
  }
  for(unsigned j=0;j<6;++j) EXPECT_NEAR(plan.trajectory.points.back().positions[j],target[j],1e-9);
}
TEST(WristSpeed, CancellationInsideTimeProfileStopsComputation) {
  auto c=context();c.simulation_only=true;c.seed[4]=0;auto target=c.seed;target[3]+=.4;
  std::array<int64_t,6> codes;codes.fill(-1);bool profile=false;unsigned checks=0;
  EXPECT_THROW(compute_preview(c,target,0,codes,[&](){return profile && ++checks>10;},
    [&](float,const std::string &phase){if(phase.find("Zwalnianie")!=std::string::npos || phase.find("zwalnianie")!=std::string::npos)profile=true;}),std::runtime_error);
  EXPECT_TRUE(profile);EXPECT_GT(checks,10u);
}
TEST(TcpFeed, LinearMillimetersPerSecondAndAccelerationApplyBetweenPoints) {
  auto c=context();c.simulation_only=true;c.seed[4]=.8;auto target=c.seed;target[0]+=.02;
  std::array<int64_t,6> codes;codes.fill(-1);const TcpLimits limits{.005,.05,.4};
  const auto plan=compute_preview(c,target,1,codes,{}, {},limits);
  robot_kinematics::Model model(c.urdf,c.lower,c.upper);check_interpolation(plan,c,model);
  const double h=1e-4,total=time_of(plan.trajectory.points.back());double peak=0;
  const double distance=(model.fk(target).translation()-model.fk(c.seed).translation()).norm();
  EXPECT_GE(total,distance/limits.velocity);
  for(double t=.001;t<total-.001;t+=.005) {
    const auto p=model.fk(position_at(plan,t)).translation().eval();
    const auto a=model.fk(position_at(plan,t-h)).translation().eval(),b=model.fk(position_at(plan,t+h)).translation().eval();
    const double speed=(b-a).norm()/(2*h);peak=std::max(peak,speed);
    EXPECT_LE(speed,limits.velocity+1e-6);
    EXPECT_LE((b-2*p+a).norm()/(h*h),limits.acceleration+1e-3);
  }
  EXPECT_GT(peak,limits.velocity*.9);
  const auto faster=compute_preview(c,target,1,codes,{}, {},TcpLimits{.01,.05,.4});
  EXPECT_LT(time_of(faster.trajectory.points.back()),total*.8);
  for(unsigned j=0;j<6;++j) EXPECT_NEAR(plan.trajectory.points.back().positions[j],target[j],1e-9);
}
TEST(TcpFeed, PureOrientationUsesSeparateAngularVelocity) {
  auto c=context();c.simulation_only=true;c.seed[4]=.8;auto target=c.seed;target[5]+=.2;
  std::array<int64_t,6> codes;codes.fill(-1);const TcpLimits limits{.02,.1,2*rad};
  const auto plan=compute_preview(c,target,1,codes,{}, {},limits);
  robot_kinematics::Model model(c.urdf,c.lower,c.upper);const auto origin=model.fk(c.seed).translation().eval();
  const double total=time_of(plan.trajectory.points.back()),h=1e-4;double peak=0;
  EXPECT_GE(total,.2/limits.angular_velocity);
  for(double t=.001;t<total-.001;t+=.01) {
    const auto a=model.fk(position_at(plan,t-h)),b=model.fk(position_at(plan,t+h));
    const double speed=Eigen::AngleAxisd(b.rotation()*a.rotation().transpose()).angle()/(2*h);
    EXPECT_LE(speed,limits.angular_velocity+1e-5);peak=std::max(peak,speed);
    EXPECT_LE((a.translation()-origin).norm(),1e-4);
  }
  EXPECT_GT(peak,limits.angular_velocity*.85);
  EXPECT_THROW(compute_preview(c,target,1,codes,{}, {},TcpLimits{NAN,.1,.1}),std::invalid_argument);
  EXPECT_THROW(compute_preview(c,target,1,codes,{}, {},TcpLimits{.1,0,.1}),std::invalid_argument);
  EXPECT_THROW(compute_preview(c,target,0,codes,{}, {},limits),std::invalid_argument);
}
TEST(PhysicalPtp, ReplansPhysicalContextAndMarksMissingReferencesExplicitly) {
  auto c=context();std::array<int64_t,6> codes;codes.fill(-1);
  for(unsigned j=0;j<6;++j) c.profiles[j].configured=true;
  auto target=c.seed;target[0]+=.05;
  const auto p=compute_physical_ptp(c,target,codes);
  EXPECT_FALSE(p.simulation_only);EXPECT_FALSE(p.execution_ready);EXPECT_EQ(p.motion_type,"PTP");
  EXPECT_NEAR(p.trajectory.points.back().positions[0],target[0],1e-9);
  c.simulated_start[5]=true;
  EXPECT_THROW(compute_physical_ptp(c,target,codes),std::invalid_argument);
  c.allow_missing_execution=true;
  const auto partial=compute_physical_ptp(c,target,codes);EXPECT_TRUE(partial.simulated_start[5]);
  c.simulation_only=true;EXPECT_THROW(compute_physical_ptp(c,target,codes),std::invalid_argument);
}
