#include "robot_motion_planner/planner.hpp"
#include "robot_control_interfaces/action/compute_cartesian_path.hpp"
#include "robot_control_interfaces/action/compute_preview_motion.hpp"
#include "robot_control_interfaces/action/compute_physical_ptp.hpp"
#include "robot_control_interfaces/msg/robot_configuration.hpp"
#include "robot_control_interfaces/srv/solve_ik.hpp"
#include "robot_control_interfaces/msg/manual_state.hpp"
#include <atomic>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

namespace robot_motion_planner {
using Action=robot_control_interfaces::action::ComputeCartesianPath;
using Handle=rclcpp_action::ServerGoalHandle<Action>;
using PreviewAction=robot_control_interfaces::action::ComputePreviewMotion;
using PhysicalAction=robot_control_interfaces::action::ComputePhysicalPtp;
using PhysicalHandle=rclcpp_action::ServerGoalHandle<PhysicalAction>;
using PreviewHandle=rclcpp_action::ServerGoalHandle<PreviewAction>;
class PlannerNode : public rclcpp::Node {
 public:
  PlannerNode():Node("robot_motion_planner") {
    const auto values=declare_parameter<std::vector<int64_t>>("mks_acceleration_codes",std::vector<int64_t>(6,-1));
    if(values.size()!=6) throw std::invalid_argument("Wymagane sześć kodów acc; -1 oznacza brak potwierdzenia");
    std::copy(values.begin(),values.end(),codes_.begin());
    description_=create_subscription<std_msgs::msg::String>("/robot/preview/robot_description",rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr value) { std::lock_guard<std::mutex> lock(mutex_); xml_=value->data; model_id_=robot_kinematics::model_id(xml_); });
    manual_=create_subscription<robot_control_interfaces::msg::ManualState>("/robot/manual/state",1,
      [this](robot_control_interfaces::msg::ManualState::ConstSharedPtr value) { std::lock_guard<std::mutex> lock(mutex_); state_=value; });
    configuration_=create_subscription<robot_control_interfaces::msg::RobotConfiguration>("/robot/configuration/active",rclcpp::QoS(1).transient_local(),
      [this](robot_control_interfaces::msg::RobotConfiguration::ConstSharedPtr value) {
        std::lock_guard<std::mutex> lock(mutex_); revision_=value->revision; configuration_received_=std::chrono::steady_clock::now(); });
    ik_=create_service<robot_control_interfaces::srv::SolveIK>("/robot/planning/solve_ik",
      [this](const robot_control_interfaces::srv::SolveIK::Request::SharedPtr request,robot_control_interfaces::srv::SolveIK::Response::SharedPtr response) {
        try {
          if(busy_) throw std::runtime_error("Planner oblicza inną drogę");
          validate_context(request->context); guard(request->context);
          robot_kinematics::Model model(request->context.urdf,request->context.lower,request->context.upper);
          if(request->forward_only) {
            response->tcp=pose_message(model.fk(request->context.seed)); response->positions=request->context.seed;
            response->success=true; response->message="FK pozycji startowej"; return;
          }
          auto answer=model.solve(pose_from_message(request->target),request->context.seed,request->context.locked,request->position_tolerance,request->orientation_tolerance,
            [&]() { return !context_valid(request->context); });
          response->success=answer.success; response->message=answer.message; response->positions=answer.positions;
          response->position_error=answer.position_error; response->orientation_error=answer.orientation_error; response->tcp=pose_message(model.fk(answer.positions));
          guard(request->context);
        } catch(const std::exception &e) { response->success=false; response->message=e.what(); }
      });
    action_=rclcpp_action::create_server<Action>(this,"/robot/planning/compute_cartesian_path",
      [this](const rclcpp_action::GoalUUID &,std::shared_ptr<const Action::Goal> goal) {
        try { validate_context(goal->context); guard(goal->context); } catch(...) { return rclcpp_action::GoalResponse::REJECT; }
        if(goal->targets.empty() || goal->targets.size()>32 || busy_.exchange(true)) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }, [](const std::shared_ptr<Handle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const std::shared_ptr<Handle> handle) { if(worker_.joinable()) worker_.join(); worker_=std::thread([this,handle]() { execute(handle); }); });
    preview_=rclcpp_action::create_server<PreviewAction>(this,"/robot/planning/compute_preview_motion",
      [this](const rclcpp_action::GoalUUID &,std::shared_ptr<const PreviewAction::Goal> goal) {
        try { validate_context(goal->context); guard(goal->context); } catch(...) { return rclcpp_action::GoalResponse::REJECT; }
        if(!goal->context.simulation_only || goal->motion_type>1 || busy_.exchange(true)) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }, [](const std::shared_ptr<PreviewHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const std::shared_ptr<PreviewHandle> handle) {
        if(worker_.joinable()) worker_.join(); worker_=std::thread([this,handle]() { execute_preview(handle); }); });
    physical_=rclcpp_action::create_server<PhysicalAction>(this,"/robot/planning/compute_physical_ptp",
      [this](const rclcpp_action::GoalUUID &,std::shared_ptr<const PhysicalAction::Goal> goal) {
        try { validate_context(goal->context);guard(goal->context); } catch(...) { return rclcpp_action::GoalResponse::REJECT; }
        if(goal->context.simulation_only || busy_.exchange(true)) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },[](const std::shared_ptr<PhysicalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const std::shared_ptr<PhysicalHandle> handle) {
        if(worker_.joinable()) worker_.join();
        worker_=std::thread([this,handle] {
          auto result=std::make_shared<PhysicalAction::Result>();
          try {
            const auto goal=handle->get_goal();
            result->plan=compute_physical_ptp(goal->context,goal->target_positions,codes_,
              [&] { return stopping_ || handle->is_canceling() || !context_valid(goal->context); },
              [&](float progress,const std::string &phase) { auto feedback=std::make_shared<PhysicalAction::Feedback>();feedback->progress=progress;feedback->phase=phase;handle->publish_feedback(feedback); });
            guard(goal->context);result->success=true;result->message="Fizyczny PTP obliczony; wymagane przygotowanie wykonawcy";
            if(handle->is_canceling()) handle->canceled(result);else handle->succeed(result);
          } catch(const std::exception &e) { result->message=e.what();if(handle->is_canceling()) handle->canceled(result);else handle->abort(result); }
          busy_=false;
        });
      });
  }
  ~PlannerNode() override { stopping_=true; if(worker_.joinable()) worker_.join(); }
 private:
  void guard(const Context &context) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(model_id_!=context.model_id || xml_!=context.urdf)
      throw std::runtime_error("Model lub rewizja konfiguracji zmieniły się; ponów od aktualnej pozycji");
    if(context.simulation_only) {
      if(revision_!=context.configuration_revision || std::chrono::steady_clock::now()-configuration_received_>=std::chrono::milliseconds(1500))
        throw std::runtime_error("Nieaktualna konfiguracja symulacji");
      return;
    }
    if(!state_ || state_->configuration_revision!=context.configuration_revision) throw std::runtime_error("Zmieniona rewizja sterowania");
    const double now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    const double elapsed=now-state_->monotonic_ns*1e-9;
    if(elapsed<0 || elapsed>=.5) throw std::runtime_error("Nieaktualny stan sterowania");
    for(unsigned i=0;i<6;++i) if(!context.locked[i]) {
      const auto &axis=state_->axes[i];
      if(context.allow_missing_execution && context.simulated_start[i] && (!axis.available || !axis.position_known)) continue;
      if(axis.axis!=i+1 || !axis.available || !axis.position_known || axis.position_age_seconds<0 || axis.position_age_seconds+elapsed>=.25 ||
        (axis.joint_configured && (!axis.calibration_valid || !axis.joint_position_known))) throw std::runtime_error("Utracono pozycję dostępnej osi; planowanie przerwane");
    }
  }
  bool context_valid(const Context &c) { try { guard(c); return !stopping_; } catch(...) { return false; } }
  void execute_preview(const std::shared_ptr<PreviewHandle> &handle) {
    auto result=std::make_shared<PreviewAction::Result>();
    try {
      const auto goal=handle->get_goal();
      result->plan=compute_preview(goal->context,goal->target_positions,goal->motion_type,codes_,
        [&]() { return stopping_ || handle->is_canceling() || !context_valid(goal->context); },
        [&](float progress,const std::string &phase) { auto f=std::make_shared<PreviewAction::Feedback>(); f->progress=progress; f->phase=phase; handle->publish_feedback(f); },
        TcpLimits{goal->tcp_velocity,goal->tcp_acceleration,goal->tcp_angular_velocity});
      guard(goal->context); result->success=true; result->message="Tylko symulacja";
      if(handle->is_canceling()) { result->success=false; handle->canceled(result); } else handle->succeed(result);
    } catch(const std::exception &e) {
      result->message=e.what(); if(handle->is_canceling()) handle->canceled(result); else handle->abort(result);
    }
    busy_=false;
  }
  void execute(const std::shared_ptr<Handle> &handle) {
    auto result=std::make_shared<Action::Result>();
    try {
      const auto goal=handle->get_goal(); std::vector<Pose> targets;
      for(const auto &target:goal->targets) targets.push_back(pose_from_message(target));
      result->plan=compute(goal->context,targets,goal->blend_position_tolerance,goal->blend_orientation_tolerance,goal->velocity_scaling,goal->acceleration_scaling,codes_,
        [&]() { return stopping_ || handle->is_canceling() || !context_valid(goal->context); },
        [&](float progress,const std::string &phase) { auto feedback=std::make_shared<Action::Feedback>(); feedback->progress=progress; feedback->phase=phase; handle->publish_feedback(feedback); });
      guard(goal->context); result->success=true; result->message="Plan gotowy do podglądu, bez wykonania";
      if(handle->is_canceling()) { result->success=false; handle->canceled(result); } else handle->succeed(result);
    } catch(const std::exception &e) {
      result->success=false; result->message=e.what(); if(handle->is_canceling()) handle->canceled(result); else handle->abort(result);
    }
    busy_=false;
  }
  std::mutex mutex_; std::atomic<bool> busy_{false},stopping_{false}; std::thread worker_;
  std::string xml_,model_id_; std::array<int64_t,6> codes_;
  std::string revision_; std::chrono::steady_clock::time_point configuration_received_{};
  rclcpp::Subscription<robot_control_interfaces::msg::RobotConfiguration>::SharedPtr configuration_;
  robot_control_interfaces::msg::ManualState::ConstSharedPtr state_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_;
  rclcpp::Subscription<robot_control_interfaces::msg::ManualState>::SharedPtr manual_;
  rclcpp::Service<robot_control_interfaces::srv::SolveIK>::SharedPtr ik_;
  rclcpp_action::Server<Action>::SharedPtr action_;
  rclcpp_action::Server<PreviewAction>::SharedPtr preview_;
  rclcpp_action::Server<PhysicalAction>::SharedPtr physical_;
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv); auto node=std::make_shared<robot_motion_planner::PlannerNode>();
  rclcpp::spin(node); node.reset(); rclcpp::shutdown(); return 0;
}
