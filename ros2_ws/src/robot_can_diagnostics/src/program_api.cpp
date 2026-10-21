#include "robot_can_diagnostics/program_api.hpp"
#include "mks_servo_can/trace.hpp"
#include <stdexcept>
#include <openssl/sha.h>
#include <sstream>
#include <iomanip>
namespace robot_can_diagnostics {
namespace {
robot_control_interfaces::msg::MotionProgramStatus message(const mks_servo_can::MotionProgramStatus &s) {
  robot_control_interfaces::msg::MotionProgramStatus out;
  out.program_id=s.program_id;out.state=s.state;out.mode=s.mode;out.message=s.message;out.execution_key=s.execution_key;
  out.participating=s.participating;out.tcp_verified=s.tcp_verified;out.busy=s.busy;out.progress=s.progress;out.elapsed=s.elapsed;out.duration=s.duration;
  out.start_positions=s.start;out.target_positions=s.target;
  return out;
}
}
ProgramApi::ProgramApi(rclcpp::Node *node,std::shared_ptr<mks_servo_can::BusRuntime> runtime):runtime_(std::move(runtime)) {
  mks_servo_can::BusConfig authorization;
  authorization.program_enabled=node->declare_parameter("execution_enabled",false);
  authorization.program_revision=node->declare_parameter("execution_configuration_revision",std::string{});
  authorization.program_stop_verified=node->declare_parameter("execution_stop_verified",false);
  authorization.program_sync_verified=node->declare_parameter("execution_sync_verified",false);
  authorization.program_staged_reset_verified=node->declare_parameter("execution_staged_reset_verified",false);
  const auto errors=node->declare_parameter("execution_tracking_error_rad",std::vector<double>(6,-1));
  const auto codes=node->declare_parameter("execution_acceleration_codes",std::vector<int64_t>(6,-1));
  if(errors.size()!=6 || codes.size()!=6) throw std::invalid_argument("Wymagane sześć tolerancji i kodów acc wykonawcy");
  for(unsigned j=0;j<6;++j) { authorization.axes[j].tracking_error=errors[j];authorization.axes[j].acceleration_code=codes[j]>=1 && codes[j]<=255 ? codes[j] : 0; }
  runtime_->configure_program_authorization(authorization);
  settings_=node->create_subscription<robot_control_interfaces::msg::MotionSettings>("/robot/motion/settings",rclcpp::QoS(1).transient_local(),
      [this](robot_control_interfaces::msg::MotionSettings::ConstSharedPtr value) { runtime_->update_motion_settings(value->revision,value->allow_missing_motion); });
  model_=node->create_subscription<std_msgs::msg::String>("/robot/preview/robot_description",rclcpp::QoS(1).transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr value) {
      if(value->data.empty() || value->data.size()>262144) { runtime_->update_motion_model("");return; }
      unsigned char bytes[SHA256_DIGEST_LENGTH];SHA256(reinterpret_cast<const unsigned char *>(value->data.data()),value->data.size(),bytes);
      std::ostringstream id;for(auto byte:bytes) id<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(byte);
      runtime_->update_motion_model(id.str());
    });
  using Prepare=robot_control_interfaces::srv::PrepareMotionProgram;
  prepare_=node->create_service<Prepare>("/robot/motion/prepare_program",[this](std::shared_ptr<Prepare::Request> request,std::shared_ptr<Prepare::Response> response) {
    try {
      const auto now=static_cast<uint64_t>(mks_servo_can::trace_ns(mks_servo_can::Clock::now()));
      if(!request->issued_monotonic_ns || request->issued_monotonic_ns>now || now-request->issued_monotonic_ns>3000000000ULL)
        throw std::runtime_error("Przeterminowane przygotowanie programu");
      const auto &plan=request->plan;
      if(plan.simulation_only || plan.motion_type!="PTP" || plan.trajectory.points.size()>10000 || plan.trajectory.joint_names.size()!=6)
        throw std::runtime_error("Wymagany fizyczny plan PTP z sześcioma osiami");
      mks_servo_can::PreparedMotionProgram input;input.id=plan.id;input.revision=plan.configuration_revision;input.model_id=plan.model_id;input.settings_revision=request->settings_revision;
      for(unsigned j=0;j<6;++j) if(plan.trajectory.joint_names[j]!="joint_"+std::to_string(j+1)) throw std::runtime_error("Nieprawidłowa kolejność osi");
      input.points.reserve(plan.trajectory.points.size());
      for(const auto &point:plan.trajectory.points) {
        if(point.positions.size()!=6 || point.velocities.size()!=6 || point.accelerations.size()!=6 || point.time_from_start.sec<0 || point.time_from_start.nanosec>=1000000000)
          throw std::runtime_error("Niepełna próbka q/v/a programu");
        mks_servo_can::ProgramPoint out;out.seconds=point.time_from_start.sec+point.time_from_start.nanosec*1e-9;
        std::copy(point.positions.begin(),point.positions.end(),out.positions.begin());std::copy(point.velocities.begin(),point.velocities.end(),out.velocities.begin());
        std::copy(point.accelerations.begin(),point.accelerations.end(),out.accelerations.begin());input.points.push_back(out);
      }
      response->status=message(runtime_->prepare_program(std::move(input)));response->accepted=true;response->message=response->status.message;
    } catch(const std::exception &e) { response->accepted=false;response->message=e.what(); }
  });
  using Get=robot_control_interfaces::srv::GetMotionProgram;
  status_=node->create_service<Get>("/robot/motion/program_status",[this](std::shared_ptr<Get::Request>,std::shared_ptr<Get::Response> response) { response->status=message(runtime_->program_status()); });
  action_=rclcpp_action::create_server<Action>(node,"/robot/motion/execute_program",
      [this](const rclcpp_action::GoalUUID &,std::shared_ptr<const Action::Goal> goal) {
        if(handle_ || !runtime_->execute_program(goal->program_id,goal->execution_key)) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },[this](std::shared_ptr<Handle>) { runtime_->stop("Anulowanie programu operatora");return rclcpp_action::CancelResponse::ACCEPT; },
      [this](std::shared_ptr<Handle> handle) { handle_=handle; });
  timer_=node->create_wall_timer(std::chrono::milliseconds(50),[this] {
    if(!handle_) return;
    const auto state=runtime_->program_status();auto feedback=std::make_shared<Action::Feedback>();feedback->status=message(state);handle_->publish_feedback(feedback);
    if(!state.busy) {
      auto result=std::make_shared<Action::Result>();result->success=state.state=="succeeded";result->message=state.message;
      if(handle_->is_canceling()) handle_->canceled(result);else if(result->success) handle_->succeed(result);else handle_->abort(result);
      handle_.reset();
    }
  });
}
}
