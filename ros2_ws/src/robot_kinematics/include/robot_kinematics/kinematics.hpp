#pragma once
#include <array>
#include <functional>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/chain.hpp>

namespace robot_kinematics {
using Joints = std::array<double, 6>;
using Mask = std::array<bool, 6>;
using Pose = Eigen::Isometry3d;
struct Solution {
  bool success{};
  std::string message;
  Joints positions{};
  double position_error{}, orientation_error{};
};
std::string model_id(const std::string &xml);
Pose interpolate(const Pose &a, const Pose &b, double t);
class Model {
 public:
  Model(const std::string &xml, const Joints &lower, const Joints &upper);
  Pose fk(const Joints &positions) const;
  Eigen::Matrix<double, 6, 6> jacobian(const Joints &positions) const;
  Solution solve(const Pose &target, const Joints &seed, const Mask &locked,
                 double position_tolerance = 1e-4, double orientation_tolerance = 1e-3,
                 const std::function<bool()> &cancelled = {}) const;
  const Joints &lower() const { return lower_; }
  const Joints &upper() const { return upper_; }
  const std::string &base() const { return base_; }
  const std::string &tip() const { return tip_; }
 private:
  KDL::Chain chain_;
  Joints lower_, upper_;
  std::string base_, tip_;
};
}
