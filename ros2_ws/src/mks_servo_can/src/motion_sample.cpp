#include "mks_servo_can/motion_sample.hpp"
#include "mks_servo_can/bus_manager.hpp"
#include <cmath>
#include <stdexcept>
namespace mks_servo_can {
MotorSample compile_motor_sample(const AxisConfig &axis,double q,double velocity,double previous,double dt,bool /*strict_speed*/) {
  if(!std::isfinite(previous) || !std::isfinite(dt) || dt<=0) throw std::invalid_argument("Nieprawidłowa próbka czasu MKS");
  const auto point=streaming_setpoint(axis,q,velocity);
  MotorSample out;out.ticks=point.ticks;out.rpm=point.rpm;out.acceleration=axis.streaming.acceleration;
  out.position_error=std::abs(CanBusManager::position_to_joint(axis,out.ticks)-point.lookahead_position);
  out.rpm_error=std::abs(out.rpm-std::abs(velocity)*axis.gear_ratio*60/(2*3.14159265358979323846)*axis.streaming.velocity_ff_scale);
  out.operation=out.ticks!=CanBusManager::joint_to_position(axis,previous);
  out.payload.assign(point.bytes.begin(),point.bytes.end());return out;
}
}
