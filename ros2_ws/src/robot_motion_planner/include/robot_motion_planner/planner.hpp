#pragma once
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include "robot_kinematics/kinematics.hpp"
#include "robot_control_interfaces/msg/planning_context.hpp"
#include "robot_control_interfaces/msg/cartesian_plan.hpp"

namespace robot_motion_planner {
using Context=robot_control_interfaces::msg::PlanningContext;
using Plan=robot_control_interfaces::msg::CartesianPlan;
using Pose=robot_kinematics::Pose;
struct TcpLimits { double velocity{},acceleration{},angular_velocity{}; };
Pose pose_from_message(const geometry_msgs::msg::Pose &message);
geometry_msgs::msg::Pose pose_message(const Pose &pose);
void validate_context(const Context &context);
// 0: osobliwość nadgarstka (J4/J6 do 5°/s), 1: pełny limit profilu.
double wrist_singularity_blend(const robot_kinematics::Model &model, const robot_kinematics::Joints &positions);
Plan compute_physical_motion(const Context &context, const robot_kinematics::Joints &target, uint8_t motion_type,
             const std::array<int64_t,6> &acceleration_codes, const std::function<bool()> &cancelled={},
             const std::function<void(float,const std::string&)> &progress={}, const TcpLimits &tcp={});
Plan compute_physical_ptp(const Context &context, const robot_kinematics::Joints &target,
             const std::array<int64_t,6> &acceleration_codes, const std::function<bool()> &cancelled={},
             const std::function<void(float,const std::string&)> &progress={});
Plan compute_preview(const Context &context, const robot_kinematics::Joints &target, uint8_t motion_type,
             const std::array<int64_t,6> &acceleration_codes, const std::function<bool()> &cancelled={},
             const std::function<void(float,const std::string&)> &progress={}, const TcpLimits &tcp={});
Plan compute(const Context &context, const std::vector<Pose> &targets, double position_tolerance, double orientation_tolerance,
             double velocity_scaling, double acceleration_scaling, const std::array<int64_t,6> &acceleration_codes,
             const std::function<bool()> &cancelled={}, const std::function<void(float,const std::string&)> &progress={});
// Ciąg ruchów PTP/LIN z przejściami (promień na torze TCP) i postojami jako jedna trajektoria (motion_type "CHAIN").
struct ChainSegment {
  uint8_t motion_type{};               // 0 PTP, 1 LIN
  robot_kinematics::Joints target{};   // rad
  std::array<double,6> max_velocity{},max_acceleration{};
  TcpLimits tcp;                       // LIN: dodatnie; PTP: zera
  double blend_radius{};               // m; 0 = postój w punkcie
  std::string tag;
};
struct ChainResult { Plan plan; size_t planned_segments{}; std::vector<std::string> warnings; };
// Błąd przypisany do ruchu ciągu (indeks w wejściu).
struct ChainError : std::runtime_error { int segment; ChainError(int s,const std::string &m):std::runtime_error(m),segment(s) {} };
ChainResult compute_chain(const Context &context,const std::vector<ChainSegment> &segments,double max_duration,
             const std::array<int64_t,6> &acceleration_codes,const std::function<bool()> &cancelled={},
             const std::function<void(float,const std::string&)> &progress={});
}
