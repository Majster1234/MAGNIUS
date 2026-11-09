#include "harness.hpp"
#include "robot_motion_planner/planner.hpp"
#include <fstream>
#include <iterator>
using namespace robot_motion_planner;
namespace {
struct Cycle {
  Context c;robot_kinematics::Joints target;std::array<int64_t,6> codes{};Plan plan;
  Cycle() {
    std::ifstream input(DEMO_URDF);c.urdf={std::istreambuf_iterator<char>(input),{}};
    c.model_id=robot_kinematics::model_id(c.urdf);c.configuration_revision="bench";
    c.seed={.2,.3,-.5,.2,.2,-.1};c.known.fill(true);c.lower.fill(-3);c.upper.fill(3);
    c.max_velocity.fill(1);c.max_acceleration.fill(2);codes.fill(-1);
    for(auto &profile:c.profiles) profile.configured=true;
    target=c.seed;target[0]+=.005;target[1]+=.003;
  }
  void cycle(std::uint64_t) {
#ifdef PHYSICAL_LIN_BENCH
    plan=compute_physical_motion(c,target,1,codes,{}, {},TcpLimits{.05,.1,.17453292519943295});
#else
    c.simulation_only=false;plan=compute_physical_ptp(c,target,codes);
    c.simulation_only=true;plan=compute_preview(c,target,1,codes,{}, {},TcpLimits{.05,.1,.17453292519943295});
#endif
  }
  bool verify() const {return plan.motion_type=="LIN" && !plan.execution_ready && plan.trajectory.points.size()>=2;}
};
}
int main(int argc,char **argv) {return robot_benchmarks::main(argc,argv,[]{return Cycle{};});}
