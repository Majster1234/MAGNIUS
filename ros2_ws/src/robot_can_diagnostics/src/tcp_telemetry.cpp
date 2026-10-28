#include "robot_can_diagnostics/tcp_jog_api.hpp"
#include "robot_kinematics/feedback_alignment.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace robot_can_diagnostics {
namespace {
std::int64_t ns(mks_servo_can::Time t) {return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();}
void quote(std::ostream &out,const std::string &value) {
  out<<'"';
  for(unsigned char c:value) {
    if(c=='"' || c=='\\')out<<'\\'<<c;
    else if(c<32)out<<"\\u00"<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(c)<<std::dec<<std::setfill(' ');
    else out<<c;
  }
  out<<'"';
}
template<class T> void array(std::ostream &out,const T &values) {
  out<<'[';bool first=true;
  for(const auto &v:values) {if(!first)out<<',';first=false;if(std::isfinite(double(v)))out<<v;else out<<"null";}
  out<<']';
}
void pose(std::ostream &out,const robot_kinematics::Pose &p) {
  const Eigen::Quaterniond q(p.rotation());
  out<<"{\"position\":";array(out,std::array<double,3>{p.translation().x(),p.translation().y(),p.translation().z()});
  out<<",\"quaternion\":";array(out,std::array<double,4>{q.x(),q.y(),q.z(),q.w()});out<<'}';
}
}
void TcpJogApi::telemetry_event(const std::string &event,const std::string &detail) {
  std::ostringstream out;out<<std::setprecision(17)<<"{\"event\":";quote(out,event);
  out<<",\"monotonic_ns\":"<<ns(mks_servo_can::Clock::now())<<",\"generation\":"<<generation_<<",\"session\":";quote(out,session_);
  out<<",\"message\":";quote(out,detail);
  if(event=="start" && generator_) {
    out<<",\"q_start\":";array(out,generator_->current().positions);out<<",\"fk_start\":";pose(out,generator_->target(0));
    out<<",\"model_id\":";quote(out,model_);out<<",\"configuration_revision\":";quote(out,revision_);
    out<<",\"axis\":"<<unsigned(status_.axis)<<",\"direction\":"<<int(status_.direction)<<",\"frame\":";quote(out,status_.frame);
    out<<",\"requested_step\":"<<status_.requested_step<<",\"velocity\":"<<status_.velocity;
    out<<",\"linear_acceleration\":"<<linear_acceleration_<<",\"ik_position_tolerance\":"<<ik_position_tolerance_<<",\"ik_orientation_tolerance\":"<<ik_orientation_tolerance_;
    out<<",\"deadline_ns\":"<<ns(session_deadline_);
    out<<",\"max_jerk_m_s3\":"<<max_jerk_<<",\"joint_stop_tolerance_deg\":"<<stop_tolerance_deg_;
  }
  out<<'}';
  if(telemetry_.size()>=256) {telemetry_.pop_front();++telemetry_dropped_;}
  telemetry_.push_back(out.str());
}
void TcpJogApi::record(const mks_servo_can::ControlState &state,const robot_kinematics::TcpJogSample &sample,bool submitted,double wall_ms) {
  const auto io=runtime_->tcp_streaming_snapshot();
  robot_kinematics::Joints raw;std::array<double,6> times{};std::array<std::int64_t,6> stamps{};
  for(unsigned j=0;j<6;++j) {raw[j]=state.positions[j].value;stamps[j]=ns(state.positions[j].stamp);times[j]=stamps[j]*1e-9;}
  const auto span=std::minmax_element(stamps.begin(),stamps.end());const auto sync_ns=*span.second;
  const auto aligned=robot_kinematics::align_feedback(raw,measured_velocity_,times,sync_ns*1e-9);
  std::ostringstream out;out<<std::setprecision(17)<<std::boolalpha;
  out<<"{\"event\":\"sample\",\"monotonic_ns\":"<<ns(mks_servo_can::Clock::now())<<",\"generation\":"<<generation_<<",\"session\":";quote(out,session_);
  out<<",\"model_id\":";quote(out,model_);out<<",\"configuration_revision\":";quote(out,revision_);
  out<<",\"q_ref\":";array(out,io.reference);out<<",\"q_ref_created_ns\":"<<ns(io.reference_created);
  out<<",\"q_f5_target\":";array(out,io.target);out<<",\"shaft_rpm\":";array(out,io.shaft_rpm);
  out<<",\"io_reference_valid\":"<<io.command_available<<",\"q_planned\":";array(out,sample.positions);
  out<<",\"q_meas\":";array(out,raw);out<<",\"dq_meas\":";array(out,measured_velocity_);
  out<<",\"q_sync\":";array(out,aligned.positions);out<<",\"rx_stamp_ns\":";array(out,stamps);
  out<<",\"t_sync_ns\":"<<sync_ns<<",\"rx_span_ms\":"<<(*span.second-*span.first)*1e-6;
  out<<",\"sync_warning\":";array(out,aligned.warning);
  out<<",\"fk_ref\":";pose(out,generator_->fk(io.reference));out<<",\"fk_raw\":";pose(out,generator_->fk(raw));
  out<<",\"fk_sync\":";pose(out,generator_->fk(aligned.positions));out<<",\"fk_target\":";pose(out,generator_->target(sample.progress));
  out<<",\"s\":"<<sample.progress<<",\"v\":"<<sample.speed<<",\"a\":"<<sample.scalar_acceleration<<",\"jerk\":"<<sample.scalar_jerk;
  out<<",\"max_jerk_m_s3\":"<<max_jerk_<<",\"joint_stop_tolerance_deg\":"<<stop_tolerance_deg_;
  out<<",\"requested_step\":"<<status_.requested_step<<",\"axis\":"<<unsigned(status_.axis)<<",\"direction\":"<<int(status_.direction);
  out<<",\"submitted\":"<<submitted<<",\"done\":"<<sample.done<<",\"braking\":"<<sample.braking<<",\"ik_wall_ms\":"<<wall_ms;
  out<<",\"telemetry_dropped\":"<<telemetry_dropped_<<'}';
  if(telemetry_.size()>=256) {telemetry_.pop_front();++telemetry_dropped_;}
  telemetry_.push_back(out.str());
}
}
