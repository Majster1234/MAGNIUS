#include "robot_can_diagnostics/tcp_jog_api.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include "mks_servo_can/diagnostic_message.hpp"
#include "robot_can_diagnostics/tcp_parameters.hpp"
#include "robot_kinematics/feedback_alignment.hpp"
#include <tinyxml2.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <limits>

namespace robot_can_diagnostics {
using namespace std::chrono_literals;
using mks_servo_can::Clock;
using mks_servo_can::Time;
namespace {
// Próg ucięcia kolejki po ciszy panelu; musi być < 200 ms dzierżawy JOG TCP w CanBusManager.
constexpr auto kKeepaliveBrake=150ms;
// Generator wyprzedza wykonanie o tyle (kolejka q/v/a jak program LIN); ucięcie nie czeka na jej koniec.
constexpr double kHorizon=.2;
constexpr unsigned kMaxStepsPerTick=25;
// Budżet IK jednego kroku generatora (przekroczenie = hamowanie po sprawdzonej drodze).
constexpr auto kStepBudget=7ms;
}
TcpJogApi::TcpJogApi(rclcpp::Node *node,std::shared_ptr<mks_servo_can::BusRuntime> runtime,
    rclcpp::CallbackGroup::SharedPtr group):runtime_(std::move(runtime)) {
  node_=node;
  rclcpp::SubscriptionOptions options;options.callback_group=group;
  enabled_=node->declare_parameter("tcp_jog_enabled",false);
  linear_velocity_=node->declare_parameter("tcp_jog_max_linear_velocity",0.);
  angular_velocity_=node->declare_parameter("tcp_jog_max_angular_velocity",0.);
  linear_acceleration_=node->declare_parameter("tcp_jog_linear_acceleration",0.);
  angular_acceleration_=node->declare_parameter("tcp_jog_angular_acceleration",0.);
  ik_position_tolerance_=node->declare_parameter("tcp_jog_ik_position_tolerance",1e-9);
  ik_orientation_tolerance_=node->declare_parameter("tcp_jog_ik_orientation_tolerance",1e-8);
  max_jerk_=node->declare_parameter("tcp_jog_max_jerk",.5);
  stop_tolerance_deg_=node->declare_parameter("motor_stop_tolerance_deg",.3);
  validate_tcp_parameters(max_jerk_,stop_tolerance_deg_);
  runtime_->configure_tcp_jog(enabled_);
  parameter_callback_=node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &parameters) {
    std::lock_guard<std::mutex> lock(mutex_);
    rcl_interfaces::msg::SetParametersResult result;result.successful=false;
    auto enabled=enabled_;auto linear=linear_velocity_,angular=angular_velocity_,la=linear_acceleration_,aa=angular_acceleration_;
    auto pt=ik_position_tolerance_,rt=ik_orientation_tolerance_;
    auto jerk=max_jerk_,tolerance=stop_tolerance_deg_;
    try {
      for(const auto &p:parameters) {
        if(p.get_name()=="tcp_jog_enabled") enabled=p.as_bool();
        else if(p.get_name()=="tcp_jog_max_linear_velocity") linear=p.as_double();
        else if(p.get_name()=="tcp_jog_max_angular_velocity") angular=p.as_double();
        else if(p.get_name()=="tcp_jog_linear_acceleration") la=p.as_double();
        else if(p.get_name()=="tcp_jog_angular_acceleration") aa=p.as_double();
        else if(p.get_name()=="tcp_jog_ik_position_tolerance") pt=p.as_double();
        else if(p.get_name()=="tcp_jog_ik_orientation_tolerance") rt=p.as_double();
        else if(p.get_name()=="tcp_jog_max_jerk") jerk=p.as_double();
        else if(p.get_name()=="motor_stop_tolerance_deg") tolerance=p.as_double();
      }
      for(double value:{linear,angular,la,aa}) if(!std::isfinite(value) || value<0) throw std::runtime_error("Limity TCP muszą być skończone i nieujemne; 0 oznacza brak konfiguracji");
      if((la>0 && linear/la>10) || (aa>0 && angular/aa>10)) throw std::runtime_error("Hamowanie TCP dłuższe niż 10 s");
      if(!std::isfinite(pt) || pt<=0 || pt>1e-4 || !std::isfinite(rt) || rt<=0 || rt>1e-3)
        throw std::runtime_error("Tolerancje IK TCP poza zakresem (0, 0.1 mm] / (0, 0.001 rad]");
      validate_tcp_parameters(jerk,tolerance);
      const bool changed=linear!=linear_velocity_ || angular!=angular_velocity_ || la!=linear_acceleration_ || aa!=angular_acceleration_ || pt!=ik_position_tolerance_ || rt!=ik_orientation_tolerance_ || jerk!=max_jerk_ || tolerance!=stop_tolerance_deg_;
      const auto manual=runtime_->manual_snapshot();
      bool moving=manual.busy;
      const auto parameter_time=Clock::now();
      for(const auto &axis:manual.axes) moving=moving || (axis.speed.fresh(parameter_time,250ms) && !shaft_at_standstill(axis.speed.value));
      if(changed && (generator_ || moving))
        throw std::runtime_error("Zmiana limitów TCP wymaga postoju");
      if(!enabled && generator_) fail("Fizyczny JOG TCP wyłączony w ROS");
      enabled_=enabled;linear_velocity_=linear;angular_velocity_=angular;linear_acceleration_=la;angular_acceleration_=aa;
      ik_position_tolerance_=pt;ik_orientation_tolerance_=rt;
      max_jerk_=jerk;stop_tolerance_deg_=tolerance;
      runtime_->configure_tcp_jog(enabled_);result.successful=true;wake_.notify_all();
    } catch(const std::exception &e) {result.reason=e.what();}
    return result;
  });
  configuration_subscription_=node->create_subscription<robot_control_interfaces::msg::RobotConfiguration>("/robot/configuration/active",rclcpp::QoS(1).transient_local(),
    [this](robot_control_interfaces::msg::RobotConfiguration::ConstSharedPtr value) {apply_limits(*value);},options);
  status_.state="blocked";
  model_subscription_=node->create_subscription<std_msgs::msg::String>("/robot/preview/robot_description",rclcpp::QoS(1).transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex_);
      if(value->data==xml_) return;
      if(generator_) fail("Zmiana modelu podczas JOG TCP");
      xml_=value->data;geometry_ready_=false;revision_.clear();model_.clear();
      if(xml_.empty() || xml_.size()>262144) return;
      tinyxml2::XMLDocument document;
      if(document.Parse(xml_.c_str())!=tinyxml2::XML_SUCCESS) return;
      const auto root=document.FirstChildElement("robot");
      if(!root) return;
      const auto kind=root->Attribute("model_kind"),tcp=root->Attribute("tcp_configured"),revision=root->Attribute("configuration_revision");
      geometry_ready_=kind && std::string(kind)=="configured" && tcp && std::string(tcp)=="true" && revision;
      if(revision) revision_=revision;
      model_=robot_kinematics::model_id(xml_);
    },options);
  service_=node->create_service<robot_control_interfaces::srv::JogTcp>("/robot/manual/jog_tcp",
    [this](std::shared_ptr<Request> request,std::shared_ptr<robot_control_interfaces::srv::JogTcp::Response> response) {
      std::lock_guard<std::mutex> lock(mutex_);command(*request,*response);
    },rmw_qos_profile_services_default,group);
  publisher_=node->create_publisher<Status>("/robot/manual/tcp_status",rclcpp::QoS(1));
  telemetry_publisher_=node->create_publisher<std_msgs::msg::String>("/robot/manual/tcp_telemetry",rclcpp::QoS(100).best_effort());
  timer_=node->create_wall_timer(50ms,[this] {
    // Konfiguracja przychodzi raz (transient local); odrzucone limity ponawiamy do skutku.
    std::optional<robot_control_interfaces::msg::RobotConfiguration> pending;
    {std::lock_guard<std::mutex> lock(mutex_);pending=pending_limits_;}
    if(pending) apply_limits(*pending);
    std::lock_guard<std::mutex> lock(mutex_);const auto now=Clock::now();
    status_.monotonic_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    status_.ready=false;status_.model_id=model_;status_.configuration_revision=revision_;
    status_.max_linear_velocity=std::isfinite(linear_velocity_) ? linear_velocity_ : 0;
    status_.max_angular_velocity=std::isfinite(angular_velocity_) ? angular_velocity_ : 0;
    if(!generator_) {
      const auto reason=unavailable(now);status_.ready=reason.empty();
      if(!reason.empty()) {status_.state="blocked";status_.message=reason;}
      else if(status_.state=="blocked") {status_.state="ready";status_.message="JOG TCP gotowy";}
    }
    publisher_->publish(status_);
    while(!telemetry_.empty()) {std_msgs::msg::String item;item.data=std::move(telemetry_.front());telemetry_.pop_front();telemetry_publisher_->publish(item);}
  },group);
  // Kolejka 200 ms do przodu: faza wątku względem slotów F5 nie ma znaczenia.
  worker_=std::thread([this] {
    auto deadline=Clock::now();
    while(running_) {
      {std::unique_lock<std::mutex> lock(mutex_);
        if(!enabled_) {wake_.wait(lock,[this]{return !running_ || enabled_;});deadline=Clock::now();}
        if(!running_) break;
        try {tick();} catch(const std::exception &e) {fail(e.what());}}
      deadline+=10ms;const auto now=Clock::now();if(deadline<now) deadline=now;
      std::this_thread::sleep_until(deadline);
    }
  });
}
TcpJogApi::~TcpJogApi() {
  running_=false;wake_.notify_all();if(worker_.joinable()) worker_.join();
  if(generator_) runtime_->stop("Zamknięcie kontrolera JOG TCP");
}
std::string TcpJogApi::unavailable(Time now) {
  if(!enabled_) return "Fizyczny JOG TCP wyłączony w parametrach ROS";
  for(double value:{linear_velocity_,angular_velocity_,linear_acceleration_,angular_acceleration_})
    if(!std::isfinite(value) || value<=0) return "Uzupełnij fizyczne limity prędkości i przyspieszenia TCP w parametrach ROS";
  if(linear_velocity_/linear_acceleration_>10 || angular_velocity_/angular_acceleration_>10)
    return "Profil TCP wymaga hamowania dłuższego niż limit generatora 10 s";
  if(!geometry_ready_) return "Wymagana kompletna geometria robota i jawnie zdefiniowany TCP";
  if(!limits_revision_.empty() && limits_revision_!=revision_) return "Limity TCP oczekują na model zgodny z zapisaną konfiguracją";
  const auto state=runtime_->manual_snapshot();
  now=Clock::now();  // Snapshot mógł otrzymać nowy pomiar podczas oczekiwania na mutex.
  if(state.configuration_revision!=revision_) return "Model nie odpowiada konfiguracji sterowania";
  if(state.fault) return "Awaria CAN: "+state.message;
  if(state.busy) return "Sterowanie zajęte: "+state.message;
  const auto config=runtime_->config_snapshot();
  if(config.bitrate!=1000000) return "Sześć osi JOG TCP wymaga CAN 1 Mbit/s";
  for(unsigned j=0;j<state.axes.size();++j) {
    const auto &axis=state.axes[j];
    const auto label=mks_servo_can::axis_diagnostic_label(j+1,config.axes[j].address)+": ";
    if(!axis.configured) return label+"brak kompletnej konfiguracji przegubu";
    if(!axis.calibration_valid) return label+"brak ważnej kalibracji";
    if(!axis.available) return label+"napęd niedostępny";
    if(!axis.joint_position.fresh(now,60ms)) return label+mks_servo_can::sample_diagnostic(axis.joint_position,now,60ms,"pozycja przegubu");
    if(!axis.enable.fresh(now,300ms)) return label+mks_servo_can::sample_diagnostic(axis.enable,now,300ms,"ENABLE");
    if(!axis.enable.value) return label+"ENABLE=0 — napęd wyłączony";
    if(!axis.speed.fresh(now,250ms)) return label+mks_servo_can::sample_diagnostic(axis.speed,now,250ms,"prędkość wału");
    if(!shaft_at_standstill(axis.speed.value)) return label+"brak potwierdzonego postoju; prędkość="+mks_servo_can::diagnostic_number(axis.speed.value)+" RPM, próg ±"+mks_servo_can::diagnostic_number(kStandstillShaftRpm)+" RPM";
  }
  if(!runtime_->tcp_jog_available()) return "Właściciel CAN oczekuje na zakończenie skanu, wolne sterowanie i gotowy profil napędów";
  return {};
}
void TcpJogApi::command(const Request &r,robot_control_interfaces::srv::JogTcp::Response &response) {
  const auto now=Clock::now();const auto issued=Time(std::chrono::nanoseconds(r.issued_monotonic_ns));
  if(!r.issued_monotonic_ns || issued>now || now-issued>=100ms || r.session.empty() || r.session.size()>64 || !r.sequence || r.action>Request::STOP) {
    response.message="Nieprawidłowa lub przeterminowana komenda TCP";return;
  }
  if(r.action!=Request::START) {
    response.accepted=generator_ && runtime_->tcp_jog_control(r.session,generation_,r.sequence,issued);
    if(response.accepted) keepalive_=issued;
    if(response.accepted && r.action==Request::STOP) {
      if(r.emergency) fail("Awaryjne przerwanie JOG TCP przez panel/most");else stop_requested_=true;
    }
    response.message=response.accepted ? "Komenda JOG TCP przyjęta" : "Brak pasującego wykonania JOG TCP";return;
  }
  if(generator_) {response.message="Poprzedni JOG TCP nadal trwa";return;}
  const auto reason=unavailable(now);
  if(!reason.empty()) {response.message=reason;return;}
  if(r.axis>5 || (r.direction!=1 && r.direction!=-1) || (r.frame!="base" && r.frame!="tool") ||
      r.model_id!=model_ || r.revision!=revision_ || !std::isfinite(r.velocity) || r.velocity<=0 ||
      r.velocity>(r.axis<3 ? linear_velocity_ : angular_velocity_) || !std::isfinite(r.step) ||
      (r.impulse ? r.step<=0 : r.step!=0)) {response.message="Nieprawidłowy kierunek, prędkość, krok lub model JOG TCP";return;}
  try {
    const auto config=runtime_->config_snapshot();mks_servo_can::ControlState state;
    if(!runtime_->try_read(state) || !state.configuration_ready) throw std::runtime_error("Brak spójnej pozycji startowej TCP");
    robot_kinematics::Joints lower,upper,vmax,amax,start;
    const auto measured_at=Clock::now();
    for(unsigned j=0;j<6;++j) {
      if(!state.positions[j].fresh(measured_at,60ms)) throw std::runtime_error("Nieaktualny start TCP");
      lower[j]=config.axes[j].lower;upper[j]=config.axes[j].upper;vmax[j]=config.axes[j].max_velocity;
      amax[j]=config.axes[j].max_acceleration;start[j]=state.positions[j].value;
    }
    auto generator=std::make_unique<robot_kinematics::TcpJog>(robot_kinematics::Model(xml_,lower,upper),start,vmax,amax,
      robot_kinematics::TcpJogRequest{r.axis,r.direction,r.frame=="base" ? robot_kinematics::TcpFrame::base : robot_kinematics::TcpFrame::tool,
        r.velocity,r.axis<3 ? linear_acceleration_ : angular_acceleration_,r.step,0.,ik_position_tolerance_,ik_orientation_tolerance_,
        r.axis<3 ? max_jerk_ : angular_acceleration_*max_jerk_/linear_acceleration_});
    generation_=runtime_->begin_tcp_jog(r.session,r.revision,r.model_id,r.sequence,issued);
    if(!generation_) throw std::runtime_error("Właściciel CAN odrzucił start TCP: profil napędów, świeżość lub zajęte sterowanie");
    // Pierwszy punkt kolejki: zmierzony start w postoju (t=0).
    mks_servo_can::ProgramPoint origin;origin.positions=start;
    if(!runtime_->append_tcp_points(r.session,generation_,{origin},false)) {
      runtime_->stop("JOG TCP: właściciel CAN odrzucił punkt startowy");throw std::runtime_error("Właściciel CAN odrzucił punkt startowy JOG TCP");
    }
    configuration_generation_=state.configuration_generation;session_=r.session;sequence_=0;stop_requested_=false;keepalive_=issued;keepalive_lost_=false;
    generator_=std::move(generator);step_=0;cut_=closed_=false;last_sample_=generator_->current();
    last_feedback_=state.positions;measured_velocity_.fill(std::numeric_limits<double>::quiet_NaN());
    session_deadline_=r.impulse ? Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(std::max(10.,generator_->duration()+3.))) : Time{};
    status_.busy=true;status_.state="running";status_.message="JOG TCP w ruchu";
    status_.axis=r.axis;status_.direction=r.direction;status_.frame=r.frame;status_.impulse=r.impulse;
    status_.requested_step=r.step;status_.velocity=r.velocity;status_.progress=0;status_.generation=generation_;
    response.accepted=true;response.message="JOG TCP uruchomiony";
    telemetry_event("start",status_.message);
  } catch(const std::exception &e) {response.message=e.what();}
}
void TcpJogApi::apply_limits(const robot_control_interfaces::msg::RobotConfiguration &value) {
  // Bez mutex_: set_parameters wywołuje callback walidacji, który sam go zajmuje.
  const auto result=node_->set_parameters_atomically({
    rclcpp::Parameter("tcp_jog_max_linear_velocity",value.tcp_jog_max_linear_velocity),
    rclcpp::Parameter("tcp_jog_max_angular_velocity",value.tcp_jog_max_angular_velocity),
    rclcpp::Parameter("tcp_jog_linear_acceleration",value.tcp_jog_linear_acceleration),
    rclcpp::Parameter("tcp_jog_angular_acceleration",value.tcp_jog_angular_acceleration)});
  std::lock_guard<std::mutex> lock(mutex_);
  if(result.successful) {limits_revision_=value.revision;pending_limits_.reset();return;}
  if(!pending_limits_ || pending_limits_->revision!=value.revision)
    RCLCPP_WARN(node_->get_logger(),"Limity TCP rewizji %s odłożone: %s; ponawianie co 50 ms",value.revision.c_str(),result.reason.c_str());
  pending_limits_=value;
}
void TcpJogApi::fail(const std::string &message) {
  telemetry_event("fault",message);
  mks_servo_can::ControlState state;
  const bool have_state=runtime_->try_read(state);
  if(generator_ && (!have_state || (!state.fault && state.active &&
      state.source==mks_servo_can::Source::tcp_jog && state.control_generation==generation_)))
    runtime_->stop(message);
  generator_.reset();status_.busy=false;status_.state="failed";status_.message=message;
}
void TcpJogApi::tick() {
  if(!generator_) return;
  mks_servo_can::ControlState state;
  if(!runtime_->try_read(state)) return;
  const auto now=Clock::now();
  if(state.fault) throw std::runtime_error("JOG TCP: "+runtime_->manual_snapshot().message);
  if(session_deadline_!=Time{} && now>=session_deadline_) {
    const double factor=status_.axis<3 ? 1000. : 180/std::acos(-1.);
    const std::string unit=status_.axis<3 ? " mm" : "°";
    throw std::runtime_error("Przekroczony timeout impulsu TCP; postęp="+mks_servo_can::diagnostic_number(status_.progress*factor)+unit+", zadany krok="+mks_servo_can::diagnostic_number(status_.requested_step*factor)+unit);
  }
  if(!state.active || state.source!=mks_servo_can::Source::tcp_jog)
    throw std::runtime_error("JOG TCP utracił właściciela sterowania; sterowanie aktywne="+std::to_string(state.active)+", źródło="+std::to_string(static_cast<int>(state.source)));
  if(state.control_generation!=generation_)
    throw std::runtime_error("JOG TCP: zmieniła się generacja sterowania; oczekiwana="+std::to_string(generation_)+", aktualna="+std::to_string(state.control_generation));
  if(state.configuration_generation!=configuration_generation_)
    throw std::runtime_error("JOG TCP: zmieniła się generacja konfiguracji; oczekiwana="+std::to_string(configuration_generation_)+", aktualna="+std::to_string(state.configuration_generation));
  for(unsigned j=0;j<state.positions.size();++j) {
    const auto &position=state.positions[j];
    if(!position.fresh(now,60ms))
      throw std::runtime_error(mks_servo_can::axis_diagnostic_label(j+1,runtime_->config_snapshot().axes[j].address)+": JOG TCP — "+mks_servo_can::sample_diagnostic(position,now,60ms,"pozycja przegubu"));
    if(position.stamp>last_feedback_[j].stamp) {
      measured_velocity_[j]=(position.value-last_feedback_[j].value)/std::chrono::duration<double>(position.stamp-last_feedback_[j].stamp).count();
      last_feedback_[j]=position;
    }
  }
  // Brak KEEPALIVE traktujemy jak puszczenie przycisku: kolejka ucięta przed limitem 200 ms właściciela CAN.
  if(!cut_ && !stop_requested_ && keepalive_!=Time{} && now-keepalive_>=kKeepaliveBrake) {
    stop_requested_=keepalive_lost_=true;
    telemetry_event("keepalive_lost","Brak KEEPALIVE "+mks_servo_can::diagnostic_number(std::chrono::duration<double,std::milli>(now-keepalive_).count())+" ms — ucięcie kolejki TCP");
  }
  // Bez rampy hamowania: napęd kończy na celu, do którego już jedzie, i wyhamowuje własną dynamiką (acc F5).
  if(!cut_ && stop_requested_) {
    if(!runtime_->cut_tcp_jog(session_,generation_,keepalive_lost_ ? "Brak KEEPALIVE z panelu" : "STOP panelu")) throw std::runtime_error("Utrata sesji JOG TCP przy ucięciu kolejki");
    cut_=true;telemetry_event("cut",keepalive_lost_ ? "Brak KEEPALIVE" : "STOP panelu");
  }
  auto trajectory=runtime_->tcp_trajectory();
  cut_=cut_ || trajectory.cut;
  double compute_ms=0;bool submitted=false;
  if(!cut_ && !closed_) {
    std::vector<mks_servo_can::ProgramPoint> points;bool done=false;const auto begin=Clock::now();
    while(points.size()<kMaxStepsPerTick && (step_+1)*.01<=trajectory.elapsed+kHorizon+1e-9 && Clock::now()-begin<kStepBudget) {
      const auto started=Clock::now();
      last_sample_=generator_->next(false,[started] {return Clock::now()-started>=kStepBudget;});
      mks_servo_can::ProgramPoint point;point.seconds=static_cast<double>(++step_)*.01;
      point.positions=last_sample_.positions;point.velocities=last_sample_.velocities;point.accelerations=last_sample_.accelerations;
      points.push_back(point);
      if(last_sample_.done) {done=true;break;}
    }
    compute_ms=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();
    if(!points.empty()) {
      if(!runtime_->append_tcp_points(session_,generation_,points,done)) throw std::runtime_error("Właściciel CAN odrzucił punkty JOG TCP (limity, ciągłość lub horyzont)");
      submitted=true;closed_=done;
    }
  }
  status_.progress=last_sample_.progress;
  if(!cut_) {
    status_.state=!trajectory.started ? "arming" : last_sample_.braking ? "braking" : "running";
    status_.message=!trajectory.started ? "Wypełnianie kolejki TCP" : last_sample_.braking ? "Hamowanie TCP: "+generator_->stop_reason() : "JOG TCP w ruchu";
  } else if(!trajectory.done && trajectory.elapsed<trajectory.end) {
    status_.state="braking";status_.message="Kolejka TCP ucięta ("+trajectory.reason+"); napęd hamuje własną dynamiką";
  } else {
    status_.state="settling";status_.message="Dolot końcowy i potwierdzenie pozycji (jak LIN)";
  }
  if(trajectory.done && runtime_->finish_tcp_jog(session_,generation_)) {
    status_.message="Zatrzymanie TCP potwierdzone";
    if(!stop_requested_ && status_.impulse && last_sample_.progress<status_.requested_step-1e-7)
      status_.message+="; impuls zakończony wcześniej: "+generator_->stop_reason();
    else if(!stop_requested_ && !status_.impulse) status_.message+="; "+(generator_->stop_reason().empty() ? trajectory.reason : generator_->stop_reason());
    record(state,last_sample_,submitted,compute_ms);telemetry_event("stopped",status_.message);
    generator_.reset();status_.busy=false;status_.state="stopped";return;
  }
  record(state,last_sample_,submitted,compute_ms);
}
}
