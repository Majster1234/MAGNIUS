#include "mks_servo_can/motion_sample.hpp"
#include "mks_servo_can/bus_manager.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace mks_servo_can {
MotorSample compile_motor_sample(const AxisConfig &axis, double q, double velocity, double previous, double dt, bool strict_speed) {
  constexpr double pi=3.14159265358979323846;
  if(!std::isfinite(q) || !std::isfinite(previous) || !std::isfinite(velocity) || !std::isfinite(dt) || dt<=0 ||
     !std::isfinite(axis.gear_ratio) || axis.gear_ratio<=0 || (axis.direction!=1 && axis.direction!=-1) ||
     !std::isfinite(axis.offset) || !std::isfinite(axis.max_velocity) || axis.max_velocity<=0 || axis.acceleration_code==0 ||
     axis.address<1 || axis.address>2047 || q<axis.lower || q>axis.upper)
    throw std::invalid_argument("Niepełny profil MKS lub próbka poza limitami");
  MotorSample out; out.ticks=CanBusManager::joint_to_position(axis,q);
  out.acceleration=axis.acceleration_code;
  out.position_error=std::abs(CanBusManager::position_to_joint(axis,out.ticks)-q);
  const auto previous_ticks=CanBusManager::joint_to_position(axis,previous);
  const bool moving=out.ticks!=previous_ticks;
  if(moving && velocity*(q-previous)<-1e-12)
    throw std::invalid_argument("Zmiana kierunku wewnątrz próbki MKS wymaga podziału odcinka");
  const auto scale=axis.gear_ratio*60/(2*pi);
  const auto requested=std::max(std::abs(velocity),std::abs(q-previous)/dt)*scale;
  const auto maximum=std::min(3000.,std::floor(axis.max_velocity*scale+1e-9));
  if(maximum<1 || requested>axis.max_velocity*scale+1e-6 || (moving && strict_speed && requested<1))
    throw std::invalid_argument("Prędkość odcinka nieprzedstawialna w RPM MKS lub poza limitem");
  out.rpm=static_cast<std::uint16_t>(moving ? std::clamp(std::round(requested),1.,maximum) : 1.);
  out.rpm_error=moving ? std::abs(out.rpm-requested) : 0;
  out.operation=moving ? 1 : 0;
  out.payload=absolute_position_request(axis.address,out.ticks,out.rpm,out.acceleration);
  return out;
}
}
