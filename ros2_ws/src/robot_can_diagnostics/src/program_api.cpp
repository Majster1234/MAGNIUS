#include "robot_can_diagnostics/program_api.hpp"
#include "mks_servo_can/trace.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <stdexcept>
#include <openssl/sha.h>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
namespace robot_can_diagnostics {
namespace {
bool nearly(double x,double y) {return std::abs(x-y)<=1e-9*std::max(1.,std::abs(y));}
robot_control_interfaces::msg::MotionProgramStatus message(const mks_servo_can::MotionProgramStatus &s) {
  robot_control_interfaces::msg::MotionProgramStatus out;
  out.program_id=s.program_id;out.state=s.state;out.mode=s.mode;out.message=s.message;out.execution_key=s.execution_key;
  out.participating=s.participating;out.tcp_verified=s.tcp_verified;out.busy=s.busy;out.progress=s.progress;out.elapsed=s.elapsed;out.duration=s.duration;
  out.dispatch_mode=s.dispatch_mode;out.sync_cycle=s.sync_cycle;out.sync_state=s.sync_state;out.sync_ack_mask=s.sync_ack_mask;
  out.sync_due_monotonic_ns=s.sync_due_monotonic_ns;out.sync_sent_monotonic_ns=s.sync_sent_monotonic_ns;
  out.sync_confirmed_monotonic_ns=s.sync_confirmed_monotonic_ns;out.sync_lateness_ms=s.sync_lateness_ms;
  out.start_positions=s.start;out.target_positions=s.target;
  return out;
}
}
void ProgramApi::publish_tuning() {
  const auto json=[](const mks_servo_can::ExecutionTuning &t) {
    std::ostringstream out;out.precision(17);
    out<<"{\"lookahead_time_sec\":"<<t.lookahead_time_sec<<",\"velocity_ff_scale\":"<<t.velocity_ff_scale<<",\"position_kp\":"<<t.position_kp
       <<",\"integral_gain\":"<<t.integral_gain<<",\"integral_limit_rad\":"<<t.integral_limit<<",\"acceleration_code\":"<<t.acceleration_code
       <<",\"acceleration_ff_scale\":"<<t.acceleration_ff_scale<<",\"gain_schedule\":"<<(t.gain_schedule ? "true" : "false")
       <<",\"hold_period_cycles\":"<<t.hold_period_cycles<<",\"hold_rpm\":"<<t.hold_rpm
       <<",\"hold_acceleration\":"<<t.hold_acceleration<<",\"hold_tail_ms\":"<<t.hold_tail_ms
       <<",\"hold_stop_mode\":"<<t.hold_stop_mode<<",\"hold_stop_ticks\":"<<t.hold_stop_ticks
       <<",\"hold_tail_end_ms\":"<<t.hold_tail_end_ms<<",\"hold_mark_scale\":"<<t.hold_mark_scale<<",\"hold_silent\":"<<t.hold_silent
       <<",\"min_lead_shaft_deg\":"<<t.min_lead_shaft_deg<<"}";
    return out.str();
  };
  const auto same=[](const mks_servo_can::ExecutionTuning &a,const mks_servo_can::ExecutionTuning &b) {
    // Tolerancja: wartości bazowe wracają z JSON (reset z mostu) z zaokrągleniem.
    return nearly(a.lookahead_time_sec,b.lookahead_time_sec) && nearly(a.velocity_ff_scale,b.velocity_ff_scale) && nearly(a.position_kp,b.position_kp) &&
        nearly(a.integral_gain,b.integral_gain) && nearly(a.integral_limit,b.integral_limit) && a.acceleration_code==b.acceleration_code &&
        nearly(a.acceleration_ff_scale,b.acceleration_ff_scale) && a.gain_schedule==b.gain_schedule &&
        a.hold_period_cycles==b.hold_period_cycles && nearly(a.hold_rpm,b.hold_rpm) && a.hold_acceleration==b.hold_acceleration &&
        a.hold_tail_ms==b.hold_tail_ms && a.hold_stop_mode==b.hold_stop_mode && a.hold_stop_ticks==b.hold_stop_ticks &&
        a.hold_tail_end_ms==b.hold_tail_end_ms && nearly(a.hold_mark_scale,b.hold_mark_scale) && a.hold_silent==b.hold_silent &&
        nearly(a.min_lead_shaft_deg,b.min_lead_shaft_deg);
  };
  std_msgs::msg::String message;
  message.data="{\"test\":"+std::string(same(tuning_,baseline_tuning_) ? "false" : "true")+",\"active\":"+json(tuning_)+",\"baseline\":"+json(baseline_tuning_)+"}";
  tuning_publisher_->publish(message);
}
ProgramApi::ProgramApi(rclcpp::Node *node,std::shared_ptr<mks_servo_can::BusRuntime> runtime):runtime_(std::move(runtime)) {
  mks_servo_can::BusConfig authorization;
  authorization.program_enabled=node->declare_parameter("execution_enabled",false);
  const auto legacy_revision=node->declare_parameter("execution_configuration_revision",std::string{});
  if(!legacy_revision.empty())
    RCLCPP_WARN(node->get_logger(),"execution_configuration_revision jest przestarzałe i ignorowane; PTP korzysta z zatwierdzonej konfiguracji ROS");
  // Przyjmujemy stare nazwy dla zgodności plików, ale nie stanowią bramek PTP.
  const bool stop_verified=node->declare_parameter("execution_stop_verified",false);
  const bool sync_verified=node->declare_parameter("execution_sync_verified",false);
  const bool reset_verified=node->declare_parameter("execution_staged_reset_verified",false);
  authorization.program_staged_reset_verified=reset_verified;
  const auto old_errors=node->declare_parameter("execution_tracking_error_rad",std::vector<double>(6,-1));
  if(stop_verified || sync_verified || std::any_of(old_errors.begin(),old_errors.end(),[](double v){return v>0;}))
    RCLCPP_WARN(node->get_logger(),"Stare flagi odbioru i execution_tracking_error_rad nie sterują podstawowym PTP");
  if(reset_verified) RCLCPP_WARN(node->get_logger(),"Automatyczne zdjęcie blokady SYNC wymaga odebranej sprzętowo sekwencji F5 STOP; włączono execution_staged_reset_verified");
  (void)node->declare_parameter("execution_ack_timeout_ms",10); // zgodność starych YAML; nie jest bramką ruchu
  authorization.use_ack=node->declare_parameter("execution_use_ack",false);
  const auto lookahead=node->declare_parameter("execution_lookahead_time_sec",.085);
  const auto ff=node->declare_parameter("execution_velocity_ff_scale",1.0);
  const auto kp=node->declare_parameter("execution_position_kp",2.5);
  // Człon całkujący (włączany opcją konfiguracji robota): wzmocnienie [1/s] i limit [rad przegubu].
  const auto integral_gain=node->declare_parameter("execution_position_integral_gain",2.0);
  const auto integral_limit=node->declare_parameter("execution_position_integral_limit_rad",0.017453292519943295);
  if(!std::isfinite(integral_gain) || integral_gain<0 || integral_gain>20 || !std::isfinite(integral_limit) || integral_limit<0 || integral_limit>.0873)
    throw std::invalid_argument("Człon całkujący: wzmocnienie 0–20 1/s, limit 0–0,0873 rad (5°)");
  // Wartość bazowa; PTP/LIN i JOG TCP nadpisuje `motion.min_dither_rpm` z konfiguracji robota (program_streaming).
  const auto minimum=node->declare_parameter("execution_min_holding_rpm",1);
  const auto maximum=node->declare_parameter("execution_max_rpm",2500);
  if(minimum<1 || maximum<minimum || maximum>3000) throw std::invalid_argument("Nieprawidłowe RPM streaming");
  authorization.program_position_period=std::chrono::milliseconds(node->declare_parameter("execution_position_period_ms",10));
  const auto tolerance=node->declare_parameter("execution_position_tolerance_rad",std::vector<double>(6,mks_servo_can::kProgramPositionTolerance));
  const auto codes=node->declare_parameter("execution_acceleration_codes",std::vector<int64_t>(6,135));
  if(tolerance.size()!=6 || codes.size()!=6) throw std::invalid_argument("Wymagane sześć tolerancji pozycji i kodów acc wykonawcy");
  for(unsigned j=0;j<6;++j) {
    authorization.program_position_tolerance[j]=tolerance[j];
    if(codes[j]<1 || codes[j]>255 || !std::isfinite(lookahead) || lookahead<0 || lookahead>.2 || !std::isfinite(ff) || ff<1 || ff>3 || !std::isfinite(kp) || kp<0)
      throw std::invalid_argument("Nieprawidłowy lookahead/feed-forward/acc");
    authorization.axes[j].acceleration_code=static_cast<std::uint8_t>(codes[j]);
    auto &stream=authorization.program_streaming[j];stream.lookahead_time_sec=lookahead;stream.velocity_ff_scale=ff;stream.position_kp=kp;
    stream.integral_gain=integral_gain;stream.integral_limit=integral_limit;
    stream.min_holding_rpm=minimum;stream.max_rpm=maximum;stream.acceleration=static_cast<std::uint8_t>(codes[j]);
  }
  // Składnik przyspieszenia w prędkości F5 programów LIN oraz nastawy szybkiego PTP wybierane przy przygotowaniu programu.
  const auto acceleration_ff=node->declare_parameter("execution_acceleration_ff_scale",0.0);
  const auto gain_schedule=node->declare_parameter("execution_gain_schedule_enabled",false);
  const auto fast_velocity=node->declare_parameter("execution_fast_ptp_velocity_deg_s",21.0);
  const auto fast_lookahead=node->declare_parameter("execution_fast_ptp_lookahead_time_sec",.05);
  const auto fast_kp=node->declare_parameter("execution_fast_ptp_position_kp",12.0);
  if(!std::isfinite(acceleration_ff) || acceleration_ff<0 || acceleration_ff>2) throw std::invalid_argument("Składnik przyspieszenia F5: 0–2");
  if(!std::isfinite(fast_velocity) || fast_velocity<=0 || !std::isfinite(fast_lookahead) || fast_lookahead<0 || fast_lookahead>.2 ||
      !std::isfinite(fast_kp) || fast_kp<0 || fast_kp>20) throw std::invalid_argument("Nastawy szybkiego PTP: prędkość > 0, wyprzedzenie 0–0,2 s, Kp 0–20");
  authorization.program_acceleration_ff=acceleration_ff;
  authorization.program_gain_schedule={gain_schedule,fast_velocity*std::acos(-1.)/180,fast_lookahead,fast_kp};
  runtime_->configure_program_authorization(authorization);
  baseline_tuning_=tuning_={lookahead,ff,kp,integral_gain,integral_limit,-1,acceleration_ff,gain_schedule,1,0.};
  // Tylko w locie, do testów ucieczek przy dojeździe; plik parametrów ich nie ustawia.
  (void)node->declare_parameter("execution_hold_period_cycles",1);
  (void)node->declare_parameter("execution_hold_rpm",0.0);
  (void)node->declare_parameter("execution_hold_acceleration",-1);
  (void)node->declare_parameter("execution_hold_tail_ms",0);
  (void)node->declare_parameter("execution_hold_stop_mode",0);
  (void)node->declare_parameter("execution_hold_stop_ticks",0);
  (void)node->declare_parameter("execution_hold_tail_end_ms",0);
  (void)node->declare_parameter("execution_hold_mark_scale",1.0);
  (void)node->declare_parameter("execution_hold_silent",0);
  (void)node->declare_parameter("execution_min_lead_shaft_deg",0.0);   // minimalne wyprzedzenie celu F5 [° wału], 0 = wyłączone
  // Testowe nadpisanie kodu acc F5 (0–255, 0 = bez rampy); -1 = konfiguracja robota / plik. Tylko w locie, nie w pliku.
  (void)node->declare_parameter("execution_acceleration_override",-1);
  // /robot/motion/execution_tuning (JSON, transient local): aktywne nastawy, bazowe z pliku i flaga testu.
  tuning_publisher_=node->create_publisher<std_msgs::msg::String>("/robot/motion/execution_tuning",rclcpp::QoS(1).transient_local());
  publish_tuning();
  tuning_callback_=node->add_on_set_parameters_callback([this,node](const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;result.successful=true;
    auto candidate=tuning_;bool touched=false;
    try {
      for(const auto &p:parameters) {
        const auto &name=p.get_name();
        if(name=="execution_lookahead_time_sec") candidate.lookahead_time_sec=p.as_double();
        else if(name=="execution_velocity_ff_scale") candidate.velocity_ff_scale=p.as_double();
        else if(name=="execution_position_kp") candidate.position_kp=p.as_double();
        else if(name=="execution_position_integral_gain") candidate.integral_gain=p.as_double();
        else if(name=="execution_position_integral_limit_rad") candidate.integral_limit=p.as_double();
        else if(name=="execution_acceleration_override") candidate.acceleration_code=static_cast<int>(p.as_int());
        else if(name=="execution_acceleration_ff_scale") candidate.acceleration_ff_scale=p.as_double();
        else if(name=="execution_gain_schedule_enabled") candidate.gain_schedule=p.as_bool();
        else if(name=="execution_hold_period_cycles") {
          const auto cycles=p.as_int();
          if(cycles<1 || cycles>100) {result.successful=false;result.reason="Okres ramek trzymania: 1–100 cykli";return result;}
          candidate.hold_period_cycles=static_cast<unsigned>(cycles);
        }
        else if(name=="execution_hold_rpm") candidate.hold_rpm=p.as_double();
        else if(name=="execution_hold_mark_scale") candidate.hold_mark_scale=p.as_double();
        else if(name=="execution_min_lead_shaft_deg") candidate.min_lead_shaft_deg=p.as_double();
        else if(name=="execution_hold_acceleration" || name=="execution_hold_tail_ms" || name=="execution_hold_stop_mode" || name=="execution_hold_stop_ticks" ||
            name=="execution_hold_tail_end_ms" || name=="execution_hold_silent") {
          const auto value=p.as_int();
          if(value<-1 || value>2000) {result.successful=false;result.reason="Nastawa dolotu poza zakresem";return result;}
          if(name=="execution_hold_acceleration") candidate.hold_acceleration=static_cast<int>(value);
          else if(value<0) {result.successful=false;result.reason="Nastawa dolotu poza zakresem";return result;}
          else if(name=="execution_hold_tail_ms") candidate.hold_tail_ms=static_cast<unsigned>(value);
          else if(name=="execution_hold_stop_mode") candidate.hold_stop_mode=static_cast<unsigned>(value);
          else if(name=="execution_hold_tail_end_ms") candidate.hold_tail_end_ms=static_cast<unsigned>(value);
          else if(name=="execution_hold_silent") candidate.hold_silent=static_cast<unsigned>(value);
          else candidate.hold_stop_ticks=static_cast<unsigned>(value);
        }
        else continue;
        touched=true;
      }
    } catch(const std::exception &error) {result.successful=false;result.reason=std::string("Nieprawidłowy typ parametru: ")+error.what();return result;}
    if(!touched) return result;
    std::string reason;
    // Ręcznie ustawione Kp lub wyprzedzenie mają pierwszeństwo: test dostaje dokładnie te wartości.
    auto effective=candidate;
    effective.gain_schedule=candidate.gain_schedule && nearly(candidate.lookahead_time_sec,baseline_tuning_.lookahead_time_sec) &&
        nearly(candidate.position_kp,baseline_tuning_.position_kp);
    if(!runtime_->set_execution_tuning(effective,reason)) {result.successful=false;result.reason=reason;return result;}
    tuning_=candidate;
    RCLCPP_WARN(node->get_logger(),"Nastawy regulatora wykonania zmienione w locie: lookahead %.0f ms, FF %.3g, Kp %.3g, Ki %.3g, acc %d (plik: %.0f ms, %.3g, %.3g, %.3g)",
        tuning_.lookahead_time_sec*1000,tuning_.velocity_ff_scale,tuning_.position_kp,tuning_.integral_gain,tuning_.acceleration_code,
        baseline_tuning_.lookahead_time_sec*1000,baseline_tuning_.velocity_ff_scale,baseline_tuning_.position_kp,baseline_tuning_.integral_gain);
    publish_tuning();
    return result;
  });
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
      if(plan.simulation_only || (plan.motion_type!="PTP" && plan.motion_type!="LIN" && plan.motion_type!="CHAIN") || plan.trajectory.points.size()>10000 || plan.trajectory.joint_names.size()!=6)
        throw std::runtime_error("Wymagany fizyczny plan PTP/LIN/CHAIN z sześcioma osiami");
      mks_servo_can::PreparedMotionProgram input;input.id=plan.id;input.revision=plan.configuration_revision;input.model_id=plan.model_id;input.settings_revision=request->settings_revision;
      if(request->participation_mask>0x3f) throw std::runtime_error("Maska osi programu spoza J1–J6");
      input.participation_mask=request->participation_mask;input.motion_type=plan.motion_type;
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
      },[this](std::shared_ptr<Handle>) { runtime_->stop("Anulowanie programu operatora",false);return rclcpp_action::CancelResponse::ACCEPT; },
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
