// Integracja ROS + generator + właściciel CAN; transport jest wyłącznie atrapą.
#include <gtest/gtest.h>
#include "robot_can_diagnostics/tcp_jog_api.hpp"
#include "../../mks_servo_can/test/streaming_fixture.hpp"
#include <tinyxml2.h>
#include <fstream>
#include <cmath>
#include <iterator>
#include <mutex>
using namespace std::chrono_literals;
namespace {
class Port final : public mks_servo_can::Transport {
 public:
  streaming_test::Port port;
  void open(const std::string &name) override {port.open(name);}
  void close() override {port.close();}
  bool is_open() const override {return port.is_open();}
  unsigned bitrate() const override {return 1000000;}
  void send(std::uint16_t address,const std::vector<std::uint8_t> &data) override {port.now=mks_servo_can::Clock::now();port.send(address,data);}
  std::optional<mks_servo_can::Frame> receive() override {port.now=mks_servo_can::Clock::now();return port.receive();}
};
std::string xml() {
  std::ifstream file(DEMO_URDF);const std::string original{std::istreambuf_iterator<char>(file),{}};
  tinyxml2::XMLDocument document;document.Parse(original.c_str());auto root=document.FirstChildElement("robot");
  root->SetAttribute("configuration_revision","bench");root->SetAttribute("model_kind","configured");root->SetAttribute("tcp_configured","true");
  tinyxml2::XMLPrinter printer;document.Print(&printer);return printer.CStr();
}
}
TEST(TcpJogApi, PhysicalSessionStartsReleasesAndConfirmsStopOnFakeTransport) {
  setenv("ROS_LOCALHOST_ONLY","1",1);setenv("ROS_DOMAIN_ID","194",1);
  rclcpp::init(0,nullptr);
  auto config=streaming_test::config();auto transport=std::make_unique<Port>();
  const std::array<double,6> seed{.3,-.4,.7,.3,.6,-.2};
  for(unsigned j=0;j<6;++j) transport->port.positions[j+1]=std::llround(seed[j]*10*16384/(2*std::acos(-1.)));
  auto runtime=std::make_shared<mks_servo_can::BusRuntime>(config,std::move(transport));
  runtime->require_configuration();runtime->start(false);
  mks_servo_can::RobotProfile profile;profile.revision="bench";profile.changed.fill(true);
  for(unsigned j=0;j<6;++j) {const auto &axis=config.axes[j];profile.axes[j]={true,axis.gear_ratio,axis.direction,axis.offset,axis.lower,axis.upper,axis.max_velocity,axis.max_acceleration};}
  ASSERT_TRUE(runtime->configure(mks_servo_can::ConfigurationPhase::bootstrap,"boot","",profile).accepted);
  ASSERT_TRUE(runtime->configure(mks_servo_can::ConfigurationPhase::finish,"boot","",profile).accepted);
  const auto model=xml();runtime->update_motion_model(robot_kinematics::model_id(model));
  auto node=std::make_shared<rclcpp::Node>("tcp_jog_api_test",rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("tcp_jog_enabled",true),rclcpp::Parameter("tcp_jog_max_linear_velocity",.005),
    rclcpp::Parameter("tcp_jog_max_angular_velocity",.05),rclcpp::Parameter("tcp_jog_linear_acceleration",.05),rclcpp::Parameter("tcp_jog_angular_acceleration",.5)}));
  auto api=std::make_unique<robot_can_diagnostics::TcpJogApi>(node.get(),runtime);
  std::mutex mutex;robot_control_interfaces::msg::TcpJogStatus status;
  auto subscription=node->create_subscription<robot_control_interfaces::msg::TcpJogStatus>("/robot/manual/tcp_status",1,
    [&](robot_control_interfaces::msg::TcpJogStatus::ConstSharedPtr value) {std::lock_guard<std::mutex> lock(mutex);status=*value;});
  auto publisher=node->create_publisher<std_msgs::msg::String>("/robot/preview/robot_description",rclcpp::QoS(1).transient_local());
  auto client=node->create_client<robot_control_interfaces::srv::JogTcp>("/robot/manual/jog_tcp");
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  std::thread worker([&]{executor.spin();});
  const auto cleanup=[&] {executor.cancel();worker.join();api.reset();runtime->shutdown();node.reset();rclcpp::shutdown();};
  const auto wait=[&](const std::function<bool(const robot_control_interfaces::msg::TcpJogStatus &)> &predicate) {
    const auto deadline=std::chrono::steady_clock::now()+5s;
    while(std::chrono::steady_clock::now()<deadline) {{std::lock_guard<std::mutex> lock(mutex);if(predicate(status)) return true;}std::this_thread::sleep_for(10ms);}
    return false;
  };
  std_msgs::msg::String description;description.data=model;publisher->publish(description);
  if(!wait([](const auto &s){return s.ready;})) {std::string message;{std::lock_guard<std::mutex> lock(mutex);message=status.message;}cleanup();FAIL()<<message;return;}
  using Service=robot_control_interfaces::srv::JogTcp;
  auto request=std::make_shared<Service::Request>();request->action=Service::Request::START;request->axis=0;request->direction=1;
  request->frame="tool";request->velocity=.002;request->revision="bench";request->model_id=robot_kinematics::model_id(model);request->session="test";request->sequence=1;
  std::string response_message;
  const auto send=[&] {
    request->issued_monotonic_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    auto future=client->async_send_request(request);if(future.wait_for(2s)!=std::future_status::ready) {response_message="Timeout usługi";return false;}
    const auto response=future.get();response_message=response->message;return response->accepted;
  };
  const bool started=send();
  mks_servo_can::ControlState control;
  const bool got_control=runtime->try_read(control);
  mks_servo_can::CommandBatch foreign;foreign.source=mks_servo_can::Source::tcp_jog;
  foreign.configuration_generation=control.configuration_generation;foreign.positions.fill(1.5);
  const bool foreign_rejected=got_control && !runtime->try_submit(foreign);
  bool advanced=false;
  if(started) for(unsigned i=0;i<3;++i) {
    std::this_thread::sleep_for(50ms);request->action=Service::Request::KEEPALIVE;request->sequence=i+2;
    if(!send()) break;
    {std::lock_guard<std::mutex> lock(mutex);advanced=advanced || status.progress>0;}
  }
  request->action=Service::Request::STOP;request->sequence=5;
  const bool stopped=started && send();
  if(started && !stopped) wait([](const auto &s){return !s.busy && s.generation>0;});
  const bool confirmed=stopped && wait([](const auto &s){return !s.busy && s.state=="stopped";});
  std::string execution_message;
  {std::lock_guard<std::mutex> lock(mutex);execution_message=status.message;}
  execution_message+="; CAN: "+runtime->snapshot().message;
  cleanup();EXPECT_TRUE(started)<<response_message;EXPECT_TRUE(foreign_rejected);EXPECT_TRUE(advanced)<<execution_message;
  EXPECT_TRUE(stopped)<<response_message<<"; "<<execution_message;EXPECT_TRUE(confirmed)<<execution_message;
}
TEST(TcpJogApi, ParametersEnableDisabledWorkerAndRejectInvalidLimits) {
  setenv("ROS_LOCALHOST_ONLY","1",1);setenv("ROS_DOMAIN_ID","194",1);rclcpp::init(0,nullptr);
  auto runtime=std::make_shared<mks_servo_can::BusRuntime>(mks_servo_can::BusConfig{});
  auto node=std::make_shared<rclcpp::Node>("tcp_jog_parameters_test");
  {
    robot_can_diagnostics::TcpJogApi api(node.get(),runtime);
    EXPECT_FALSE(runtime->config_snapshot().tcp_jog_enabled);
    auto result=node->set_parameters_atomically({rclcpp::Parameter("tcp_jog_enabled",true),
      rclcpp::Parameter("tcp_jog_max_linear_velocity",.02),rclcpp::Parameter("tcp_jog_linear_acceleration",.1)});
    EXPECT_TRUE(result.successful)<<result.reason;EXPECT_TRUE(runtime->config_snapshot().tcp_jog_enabled);
    result=node->set_parameters_atomically({rclcpp::Parameter("tcp_jog_max_linear_velocity",-1.)});
    EXPECT_FALSE(result.successful);EXPECT_DOUBLE_EQ(node->get_parameter("tcp_jog_max_linear_velocity").as_double(),.02);
    result=node->set_parameters_atomically({rclcpp::Parameter("tcp_jog_ik_position_tolerance",1e-7),rclcpp::Parameter("tcp_jog_ik_orientation_tolerance",1e-6)});
    EXPECT_TRUE(result.successful)<<result.reason;
    result=node->set_parameters_atomically({rclcpp::Parameter("tcp_jog_ik_position_tolerance",1e-3)});
    EXPECT_FALSE(result.successful);EXPECT_DOUBLE_EQ(node->get_parameter("tcp_jog_ik_position_tolerance").as_double(),1e-7);
    auto publisher=node->create_publisher<robot_control_interfaces::msg::RobotConfiguration>("/robot/configuration/active",rclcpp::QoS(1).transient_local());
    robot_control_interfaces::msg::RobotConfiguration configuration;configuration.revision="configuration-test";
    configuration.tcp_jog_max_linear_velocity=.004;configuration.tcp_jog_max_angular_velocity=.05;
    configuration.tcp_jog_linear_acceleration=.1;configuration.tcp_jog_angular_acceleration=.5;
    publisher->publish(configuration);
    for(unsigned i=0;i<100 && node->get_parameter("tcp_jog_max_linear_velocity").as_double()!=.004;++i) {
      rclcpp::spin_some(node);std::this_thread::sleep_for(10ms);
    }
    EXPECT_DOUBLE_EQ(node->get_parameter("tcp_jog_max_linear_velocity").as_double(),.004);
    EXPECT_DOUBLE_EQ(node->get_parameter("tcp_jog_max_angular_velocity").as_double(),.05);
    result=node->set_parameters_atomically({rclcpp::Parameter("tcp_jog_enabled",false)});
    EXPECT_TRUE(result.successful);EXPECT_FALSE(runtime->config_snapshot().tcp_jog_enabled);
  }
  node.reset();rclcpp::shutdown();
}
