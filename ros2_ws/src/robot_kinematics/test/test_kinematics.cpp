#include <gtest/gtest.h>
#include <fstream>
#include <iterator>
#include "robot_kinematics/kinematics.hpp"
using namespace robot_kinematics;
namespace {
std::string xml() { std::ifstream input(DEMO_URDF); return {std::istreambuf_iterator<char>(input),{}}; }
Model model() { Joints lower,upper; lower.fill(-20);upper.fill(20); return Model(xml(),lower,upper); }
}
TEST(Kinematics, FKIKFullOrientationAndJacobian) {
  auto m=model(); const Joints seed{.2,.3,-.5,.2,.2,-.1}, goal{.22,.31,-.49,.21,.22,-.12};
  const auto target=m.fk(goal); auto result=m.solve(target,seed,{});
  ASSERT_TRUE(result.success)<<result.message;
  EXPECT_LT((m.fk(result.positions).translation()-target.translation()).norm(),1e-4);
  EXPECT_LT(Eigen::AngleAxisd(m.fk(result.positions).rotation()*target.rotation().transpose()).angle(),1e-3);
  const auto jac=m.jacobian(seed); const auto start=m.fk(seed);
  for(unsigned i=0;i<6;++i) { auto shifted=seed;shifted[i]+=1e-6; const auto position=m.fk(shifted);
    EXPECT_LT(((position.translation()-start.translation())/1e-6-jac.col(i).head<3>()).norm(),1e-5); }
}
TEST(Kinematics, LockedAxisStaysLockedAndUnreachableTargetFails) {
  auto m=model(); Joints seed{.3,.2,-.4,.2,.4,.1}, goal=seed; goal[0]+=.02;
  Mask locked; locked.fill(true);locked[0]=false;
  auto result=m.solve(m.fk(goal),seed,locked); ASSERT_TRUE(result.success);
  for(unsigned i=1;i<6;++i) EXPECT_EQ(result.positions[i],seed[i]);
  auto unreachable=m.fk(seed);unreachable.translation().x()=20;
  EXPECT_FALSE(m.solve(unreachable,seed,locked).success);
  EXPECT_FALSE(m.solve(m.fk(goal),seed,Mask{true,true,true,true,true,true}).success);
}
TEST(Kinematics, MultiTurnLimitsAndCancellation) {
  auto m=model(); Joints q{7.0,.2,-.3,0,.4,0}; auto result=m.solve(m.fk(q),q,{});
  ASSERT_TRUE(result.success); EXPECT_DOUBLE_EQ(result.positions[0],7.0);
  EXPECT_FALSE(m.solve(m.fk(q),q,{},1e-4,1e-3,[](){return true;}).success);
  q[0]=NAN; EXPECT_THROW(m.fk(q),std::invalid_argument);
  EXPECT_EQ(model_id(xml()).size(),64u); EXPECT_NE(model_id(xml()),model_id(xml()+" "));
}
