#pragma once
// Wewnętrzne narzędzia planera (wspólne dla pojedynczego PTP/LIN i ciągu ruchów). Nie jest instalowany.
#include "robot_motion_planner/planner.hpp"
#include <memory>
#include <unordered_map>
#include <moveit/robot_model/robot_model.h>

namespace robot_motion_planner::detail {
using Trajectory=trajectory_msgs::msg::JointTrajectory;
constexpr size_t max_points=10000;
double seconds(const builtin_interfaces::msg::Duration &duration);
builtin_interfaces::msg::Duration duration(double t);
struct Sample { robot_kinematics::Joints q{},v{},a{}; };
// Interpolacja Hermite'a 5. stopnia jak w JTC i kompilatorze MKS; `index` przesuwa się tylko do przodu.
Sample sample(const Trajectory &trajectory,double t,size_t &index);
double tcp_excess(const robot_kinematics::Model &model,const Sample &s,const TcpLimits &limits);
std::array<double,6> local_velocity_limits(const robot_kinematics::Model &model,const Sample &s,const std::array<double,6> &requested);
void stretch_time(Trajectory &trajectory,double factor);
// Limity w danej próbce drogi: osie (prędkość/przyspieszenie) i TCP (velocity=0 → bez limitu TCP).
struct SampleCaps { std::array<double,6> v{},a{}; TcpLimits tcp; };
// `reset` przed każdym przejściem po drodze (wywołania `at` idą wzdłuż drogi, monotonicznie).
struct CapProvider { std::function<void()> reset; std::function<SampleCaps(const robot_kinematics::Joints&)> at; };
// Zmiana zegara drogi pod limity per próbka. vmax/amax = limity, z którymi liczono TOTG (górne ograniczenie).
void limit_path_speed(Trajectory &trajectory,const robot_kinematics::Model &model,const std::array<double,6> &vmax,
                      const std::array<double,6> &amax,const CapProvider &caps,bool uniform,const std::function<void()> &check);
bool near_path(const Pose &pose,const std::vector<Pose> &path,double p_tol,double r_tol);
std::shared_ptr<moveit::core::RobotModel> make_moveit_model(const Context &c,const robot_kinematics::Model &kinematics);
// TOTG (odchyłka `deviation`), usunięcie punktów o tym samym czasie, kolejność joint_1..6, zapas 5% i czas
// zaokrąglony do wielokrotności cyklu 10 ms. Limity per oś w kolejności joint_1..6.
Trajectory totg(const std::shared_ptr<moveit::core::RobotModel> &model,const std::vector<robot_kinematics::Joints> &positions,
                const std::array<double,6> &vmax,const std::array<double,6> &amax,double deviation,const std::function<void()> &check);
// Ramki F5 100 Hz do podglądu MKS (plan.mks_samples / mks_compatible / mks_message).
void compile_mks(Plan &plan,const Context &c,const std::array<int64_t,6> &codes,const std::function<void()> &check);
}
