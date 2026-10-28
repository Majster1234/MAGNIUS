#pragma once
#include "robot_kinematics/kinematics.hpp"
#include <algorithm>
#include <cmath>

namespace robot_kinematics {
struct AlignedFeedback {
  Joints positions{};
  std::array<bool,6> warning{};
};
inline AlignedFeedback align_feedback(const Joints &raw,const Joints &velocity,
    const std::array<double,6> &times,double sync_time) {
  AlignedFeedback out;out.positions=raw;
  for(unsigned j=0;j<6;++j) {
    const double age=sync_time-times[j];
    if(!std::isfinite(age) || std::abs(age)>.050 || !std::isfinite(velocity[j]) || !std::isfinite(raw[j])) {out.warning[j]=true;continue;}
    out.positions[j]+=velocity[j]*std::clamp(age,-.020,.020);
  }
  return out;
}
}
