#pragma once
#include "robot_kinematics/kinematics.hpp"
#include <deque>

namespace robot_kinematics {
enum class TcpFrame { base, tool };
struct TcpJogRequest {
  unsigned axis{};  // x/y/z/rx/ry/rz = 0..5
  int direction{};
  TcpFrame frame{TcpFrame::base};
  double velocity{}, acceleration{}, step{};  // SI, step=0: ruch ciągły
  double lookahead{.085};
  double position_tolerance{1e-9}, orientation_tolerance{1e-8};  // Tylko dokładność IK, m/rad.
};
struct TcpJogSample {
  Joints positions{}, velocities{}, accelerations{}, lookahead_positions{};
  double progress{}, speed{}, remaining{};
  bool braking{}, done{};
};
// Jednowymiarowa droga TCP i zawsze gotowa, sprawdzona droga hamowania.
// Nie dotyka ROS ani transportu. Nie obcina poszczególnych przegubów.
class TcpJog {
 public:
  TcpJog(Model model, const Joints &start, const Joints &velocity_limits,
         const Joints &acceleration_limits, TcpJogRequest request);
  TcpJogSample next(bool stop = false, const std::function<bool()> &cancelled = {});
  Pose target(double progress) const;
  const TcpJogSample &current() const { return current_; }
  const std::string &stop_reason() const { return stop_reason_; }
 private:
  TcpJogSample point(const TcpJogSample &previous, double speed, double distance,
                     const std::function<bool()> &cancelled) const;
  std::deque<TcpJogSample> braking(const TcpJogSample &from,
                                  const std::function<bool()> &cancelled) const;
  Model model_;
  Joints vmax_, amax_;
  TcpJogRequest request_;
  Pose origin_;
  void set_lookahead();
  Eigen::Vector3d axis_;
  TcpJogSample current_;
  std::deque<TcpJogSample> brake_;
  bool stopping_{};
  std::string stop_reason_;
};
}
