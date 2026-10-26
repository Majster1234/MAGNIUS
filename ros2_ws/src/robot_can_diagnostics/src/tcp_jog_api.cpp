#include "robot_can_diagnostics/tcp_jog_api.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"
#include <tinyxml2.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <algorithm>
#include <cmath>

namespace robot_can_diagnostics {
using namespace std::chrono_literals;
using mks_servo_can::Clock;
using mks_servo_can::Time;
TcpJogApi::TcpJogApi(rclcpp::Node *node,std::shared_ptr<mks_servo_can::BusRuntime> runtime):runtime_(std::move(runtime)) {
  enabled_=node->declare_parameter("tcp_jog_enabled",false);
  linear_velocity_=node->declare_parameter("tcp_jog_max_linear_velocity",0.);
  angular_velocity_=node->declare_parameter("tcp_jog_max_angular_velocity",0.);
  linear_acceleration_=node->declare_parameter("tcp_jog_linear_acceleration",0.);
  angular_acceleration_=node->declare_parameter("tcp_jog_angular_acceleration",0.);
  ik_position_tolerance_=node->declare_parameter("tcp_jog_ik_position_tolerance",1e-9);
  ik_orientation_tolerance_=node->declare_parameter("tcp_jog_ik_orientation_tolerance",1e-8);
  runtime_->configure_tcp_jog(enabled_);
  parameter_callback_=node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &parameters) {
    std::lock_guard<std::mutex> lock(mutex_);
    rcl_interfaces::msg::SetParametersResult result;result.successful=false;
    auto enabled=enabled_;auto linear=linear_velocity_,angular=angular_velocity_,la=linear_acceleration_,aa=angular_acceleration_;
    auto pt=ik_position_tolerance_,rt=ik_orientation_tolerance_;
    try {
      for(const auto &p:parameters) {
        if(p.get_name()=="tcp_jog_enabled") enabled=p.as_bool();
        else if(p.get_name()=="tcp_jog_max_linear_velocity") linear=p.as_double();
        else if(p.get_name()=="tcp_jog_max_angular_velocity") angular=p.as_double();
        else if(p.get_name()=="tcp_jog_linear_acceleration") la=p.as_double();
        else if(p.get_name()=="tcp_jog_angular_acceleration") aa=p.as_double();
        else if(p.get_name()=="tcp_jog_ik_position_tolerance") pt=p.as_double();
        else if(p.get_name()=="tcp_jog_ik_orientation_tolerance") rt=p.as_double();
      }
      for(double value:{linear,angular,la,aa}) if(!std::isfinite(value) || value<0) throw std::runtime_error("Limity TCP muszą być skończone i nieujemne; 0 oznacza brak konfiguracji");
      if((la>0 && linear/la>10) || (aa>0 && angular/aa>10)) throw std::runtime_error("Hamowanie TCP dłuższe niż 10 s");
      if(!std::isfinite(pt) || pt<=0 || pt>1e-4 || !std::isfinite(rt) || rt<=0 || rt>1e-3)
        throw std::runtime_error("Tolerancje IK TCP poza zakresem (0, 0.1 mm] / (0, 0.001 rad]");
      if(generator_ && enabled && (linear!=linear_velocity_ || angular!=angular_velocity_ || la!=linear_acceleration_ || aa!=angular_acceleration_ || pt!=ik_position_tolerance_ || rt!=ik_orientation_tolerance_))
        throw std::runtime_error("Zmiana limitów TCP wymaga postoju");
      if(!enabled && generator_) fail("Fizyczny JOG TCP wyłączony w ROS");
      enabled_=enabled;linear_velocity_=linear;angular_velocity_=angular;linear_acceleration_=la;angular_acceleration_=aa;
      ik_position_tolerance_=pt;ik_orientation_tolerance_=rt;
      runtime_->configure_tcp_jog(enabled_);result.successful=true;wake_.notify_all();
    } catch(const std::exception &e) {result.reason=e.what();}
    return result;
  });
  configuration_subscription_=node->create_subscription<robot_control_interfaces::msg::RobotConfiguration>("/robot/configuration/active",rclcpp::QoS(1).transient_local(),
    [this,node](robot_control_interfaces::msg::RobotConfiguration::ConstSharedPtr value) {
      const auto result=node->set_parameters_atomically({
        rclcpp::Parameter("tcp_jog_max_linear_velocity",value->tcp_jog_max_linear_velocity),
        rclcpp::Parameter("tcp_jog_max_angular_velocity",value->tcp_jog_max_angular_velocity),
        rclcpp::Parameter("tcp_jog_linear_acceleration",value->tcp_jog_linear_acceleration),
        rclcpp::Parameter("tcp_jog_angular_acceleration",value->tcp_jog_angular_acceleration)});
      if(result.successful) {std::lock_guard<std::mutex> lock(mutex_);limits_revision_=value->revision;}
    });
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
    });
  service_=node->create_service<robot_control_interfaces::srv::JogTcp>("/robot/manual/jog_tcp",
    [this](std::shared_ptr<Request> request,std::shared_ptr<robot_control_interfaces::srv::JogTcp::Response> response) {
      std::lock_guard<std::mutex> lock(mutex_);command(*request,*response);
    });
  publisher_=node->create_publisher<Status>("/robot/manual/tcp_status",rclcpp::QoS(1));
  timer_=node->create_wall_timer(50ms,[this] {
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
  });
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
  if(state.fault || state.busy) return "Sterowanie zajęte lub awaria CAN";
  const auto config=runtime_->config_snapshot();
  if(config.bitrate!=1000000) return "Sześć osi JOG TCP wymaga CAN 1 Mbit/s";
  for(const auto &axis:state.axes) {
    if(!axis.configured || !axis.calibration_valid || !axis.available || !axis.joint_position.fresh(now,60ms) ||
        !axis.enable.fresh(now,300ms) || !axis.enable.value || !axis.speed.fresh(now,250ms) || axis.speed.value!=0)
      return "Wymagane sześć skalibrowanych osi, ENABLE i potwierdzony postój";
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
    response.accepted=generator_ && runtime_->tcp_jog_control(r.session,generation_,r.sequence,issued,r.action==Request::STOP);
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
        r.velocity,r.axis<3 ? linear_acceleration_ : angular_acceleration_,r.step,config.program_streaming[0].lookahead_time_sec,ik_position_tolerance_,ik_orientation_tolerance_});
    generation_=runtime_->begin_tcp_jog(r.session,r.revision,r.model_id,r.sequence,issued);
    if(!generation_) throw std::runtime_error("Właściciel CAN odrzucił start TCP: profil napędów, świeżość lub zajęte sterowanie");
    configuration_generation_=state.configuration_generation;session_=r.session;sequence_=0;stop_requested_=false;
    settling_=confirmed_=last_position_=brake_deadline_={};generator_=std::move(generator);
    status_.busy=true;status_.state="running";status_.message="JOG TCP w ruchu";
    status_.axis=r.axis;status_.direction=r.direction;status_.frame=r.frame;status_.impulse=r.impulse;
    status_.requested_step=r.step;status_.velocity=r.velocity;status_.progress=0;status_.generation=generation_;
    response.accepted=true;response.message="JOG TCP uruchomiony";
  } catch(const std::exception &e) {response.message=e.what();}
}
void TcpJogApi::fail(const std::string &message) {
  if(generator_) runtime_->stop(message);
  generator_.reset();status_.busy=false;status_.state="failed";status_.message=message;
}
void TcpJogApi::tick() {
  if(!generator_) return;
  mks_servo_can::ControlState state;
  if(!runtime_->try_read(state)) return;
  const auto now=Clock::now();
  if(state.fault || !state.active || state.source!=mks_servo_can::Source::tcp_jog ||
      state.control_generation!=generation_ || state.configuration_generation!=configuration_generation_)
    throw std::runtime_error("JOG TCP utracił sterowanie lub konfigurację");
  for(const auto &position:state.positions) if(!position.fresh(now,60ms)) throw std::runtime_error("Utrata pomiarów JOG TCP");
  auto sample=generator_->next(stop_requested_,[now] {return Clock::now()-now>=7ms;});
  if(!sample.braking) {
    status_.state=sample.progress==0 ? "arming" : "running";
    status_.message=sample.progress==0 ? "Oczekiwanie na obliczenie pierwszego kroku TCP" : "JOG TCP w ruchu";
  }
  if(sample.braking) {
    if(!runtime_->brake_tcp_jog(session_,generation_)) throw std::runtime_error("Utrata sesji hamowania TCP");
    if(brake_deadline_==Time{}) brake_deadline_=now+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(sample.remaining))+3s;
    if(now>=brake_deadline_) throw std::runtime_error("Nie potwierdzono zatrzymania TCP; wymagany reset");
    status_.state="braking";status_.message=stop_requested_ || generator_->stop_reason().empty() ? "Hamowanie TCP" : "Hamowanie TCP: "+generator_->stop_reason();
  }
  status_.progress=sample.progress;
  if(sample.done) {
    status_.state="settling";status_.message="Oczekiwanie na pomiarowe potwierdzenie postoju TCP";
    if(settling_==Time{}) settling_=now;
    const auto manual=runtime_->manual_snapshot();const auto confirmation_time=Clock::now();bool stopped=true;Time oldest=now;
    const auto config=runtime_->config_snapshot();
    for(unsigned j=0;j<6;++j) {
      const auto &axis=manual.axes[j];oldest=std::min(oldest,state.positions[j].stamp);
      stopped=stopped && state.positions[j].stamp>settling_ && axis.speed.fresh(confirmation_time,250ms) && axis.speed.value==0 &&
        std::abs(state.positions[j].value-sample.positions[j])<=config.program_position_tolerance[j];
    }
    if(stopped && oldest>last_position_) {
      last_position_=oldest;if(confirmed_==Time{}) confirmed_=oldest;
      else if(oldest-confirmed_>=100ms && runtime_->finish_tcp_jog(session_,generation_)) {
        status_.message="Zatrzymanie TCP potwierdzone";
        if(!stop_requested_ && status_.impulse && sample.progress<status_.requested_step-1e-7)
          status_.message+="; impuls zakończony wcześniej: "+generator_->stop_reason();
        else if(!stop_requested_ && !status_.impulse) status_.message+="; "+generator_->stop_reason();
        generator_.reset();status_.busy=false;status_.state="stopped";return;
      }
    } else if(!stopped) confirmed_={};
  }
  mks_servo_can::CommandBatch batch;batch.positions=sample.positions;batch.velocities=sample.velocities;
  batch.lookahead_positions=sample.lookahead_positions;
  batch.accelerations=sample.accelerations;batch.remaining_time_sec=sample.remaining;batch.velocities_valid=true;
  batch.created=Clock::now();batch.expires=batch.created+20ms;batch.source=mks_servo_can::Source::tcp_jog;
  batch.sequence=++sequence_;batch.configuration_generation=configuration_generation_;batch.control_generation=generation_;
  // Chwilowa kolizja publikacji lub zapisu snapshotu nie tworzy kolejki.
  // Trwałą utratę referencji zatrzymuje niezależnie watchdog IO (60 ms).
  runtime_->try_submit(batch);
}
}
