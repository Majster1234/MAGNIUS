#include <gtest/gtest.h>
#include <fstream>
#include <iterator>
#include "robot_kinematics/tcp_jog.hpp"
using namespace robot_kinematics;
namespace {
std::string xml() {std::ifstream file(DEMO_URDF);return {std::istreambuf_iterator<char>(file),{}};}
Joints filled(double value) {Joints result;result.fill(value);return result;}
const Joints start{.3,-.4,.7,.3,.6,-.2};
Model model() {return Model(xml(),filled(-3),filled(3));}
}
TEST(TcpJog, AllDirectionsFramesAndModesFollowCartesianPath) {
  for(auto frame:{TcpFrame::base,TcpFrame::tool}) for(unsigned axis=0;axis<6;++axis) for(int sign:{-1,1}) for(bool impulse:{false,true}) {
    SCOPED_TRACE(::testing::Message()<<axis<<"/"<<sign<<"/"<<int(frame)<<"/"<<impulse);
    const auto m=model();const auto initial=m.fk(start);
    TcpJog jog(m,start,filled(2),filled(10),{axis,sign,frame,.005,.05,impulse ? .001 : 0.});
    TcpJogSample sample;
    for(unsigned i=0;i<100;++i) {
      sample=jog.next(!impulse && i>=30);
      const auto actual=m.fk(sample.positions),expected=jog.target(sample.progress);
      if(impulse) {
        const auto preview=m.fk(sample.lookahead_positions);
        if(axis<3) EXPECT_LE((preview.translation()-initial.translation()).norm(),.001+1e-8);
        else EXPECT_LE(Eigen::AngleAxisd(preview.rotation()*initial.rotation().transpose()).angle(),.001+1e-7);
      }
      EXPECT_LT((actual.translation()-expected.translation()).norm(),1e-8);
      EXPECT_LT(Eigen::AngleAxisd(actual.rotation()*expected.rotation().transpose()).angle(),1e-7);
      for(unsigned j=0;j<6;++j) {EXPECT_LE(std::abs(sample.velocities[j]),2+1e-8);EXPECT_LE(std::abs(sample.accelerations[j]),10+1e-6);}
      if(axis<3) EXPECT_LT(Eigen::AngleAxisd(actual.rotation()*initial.rotation().transpose()).angle(),1e-7);
      else EXPECT_LT((actual.translation()-initial.translation()).norm(),1e-8);
      if(sample.done) break;
    }
    EXPECT_TRUE(sample.done);EXPECT_GT(sample.progress,0);
    if(impulse) {EXPECT_LE(sample.progress,.001+1e-10);EXPECT_NEAR(sample.progress,.001,1e-7);}
  }
}
TEST(TcpJog, ReleaseBrakesEarlyAndNeverResumes) {
  TcpJog jog(model(),start,filled(2),filled(10),{0,1,TcpFrame::tool,.005,.05,.02});
  for(unsigned i=0;i<10;++i) jog.next();
  const auto before=jog.current();auto sample=jog.next(true);
  for(unsigned i=0;i<50 && !sample.done;++i) sample=jog.next();
  EXPECT_TRUE(sample.done);EXPECT_LT(sample.progress,.02);EXPECT_GE(sample.progress,before.progress);
  EXPECT_DOUBLE_EQ(jog.next().progress,sample.progress);
}
TEST(TcpJog, InitialCalculationBudgetKeepsStandstillAndRetriesWithoutQueue) {
  TcpJog jog(model(),start,filled(2),filled(10),{0,1,TcpFrame::tool,.005,.05,0});
  const auto waiting=jog.next(false,[]{return true;});
  EXPECT_DOUBLE_EQ(waiting.progress,0);EXPECT_FALSE(waiting.braking);EXPECT_FALSE(waiting.done);
  EXPECT_EQ(waiting.positions,start);EXPECT_EQ(waiting.lookahead_positions,start);
  for(double v:waiting.velocities) EXPECT_DOUBLE_EQ(v,0);
  EXPECT_GT(jog.next().progress,0);
  EXPECT_TRUE(jog.next(true).braking);
}
TEST(TcpJog, LimitsAndCancellationUsePreviouslyValidatedBrake) {
  auto lower=filled(-3),upper=filled(3);upper[0]=start[0]+.001;
  TcpJog jog(Model(xml(),lower,upper),start,filled(.05),filled(.2),{1,1,TcpFrame::base,.01,.05,0});
  TcpJogSample sample;
  for(unsigned i=0;i<300 && !sample.done;++i) sample=jog.next();
  EXPECT_TRUE(sample.done);for(unsigned j=0;j<6;++j) {EXPECT_LE(sample.positions[j],upper[j]);EXPECT_GE(sample.positions[j],lower[j]);}
  TcpJog cancelled(model(),start,filled(2),filled(10),{0,1,TcpFrame::base,.005,.05,0});
  cancelled.next();sample=cancelled.next(false,[]{return true;});
  EXPECT_TRUE(sample.braking);
  EXPECT_THROW(TcpJog(model(),start,filled(2),filled(10),{6,1,TcpFrame::base,.005,.05,0}),std::invalid_argument);
}
TEST(TcpJog, ConfiguredIkToleranceStillBoundsMeasuredCartesianPathAndJointDynamics) {
  for(unsigned axis=0;axis<3;++axis) for(double tolerance:{1e-8,1e-7,1e-6}) {
    SCOPED_TRACE(::testing::Message()<<axis<<"/"<<tolerance);
    TcpJogRequest request{axis,1,TcpFrame::base,.002,.1,.01};
    request.position_tolerance=tolerance;request.orientation_tolerance=tolerance*10;
    auto m=model();TcpJog jog(m,start,filled(2),filled(10),request);TcpJogSample sample;
    for(unsigned i=0;i<1000;++i) {
      sample=jog.next();const auto actual=m.fk(sample.positions),target=jog.target(sample.progress);
      EXPECT_LE((actual.translation()-target.translation()).norm(),tolerance+1e-12);
      EXPECT_LE(Eigen::AngleAxisd(actual.rotation()*target.rotation().transpose()).angle(),10*tolerance+1e-12);
      for(unsigned j=0;j<6;++j) {EXPECT_LE(std::abs(sample.velocities[j]),2+1e-8);EXPECT_LE(std::abs(sample.accelerations[j]),10+1e-6);}
      if(sample.done)break;
    }
    EXPECT_TRUE(sample.done);EXPECT_NEAR(sample.progress,.01,1e-7);
    request.position_tolerance=1e-3;
    EXPECT_THROW(TcpJog(model(),start,filled(2),filled(10),request),std::invalid_argument);
  }
}
