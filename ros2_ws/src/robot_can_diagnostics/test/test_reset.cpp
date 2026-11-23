#include <gtest/gtest.h>
#include "robot_can_diagnostics/diagnostics.hpp"
#include <atomic>
#include <deque>
using namespace mks_servo_can;
using namespace std::chrono_literals;
namespace {
// Transport pamięciowy: ten test nie otwiera SocketCAN ani nie wywołuje systemctl.
class MemoryCan final : public Transport {
 public:
  std::atomic<unsigned> identities{0};
  std::atomic<unsigned> positions{0};
  bool opened{};
  std::deque<Frame> replies;
  void open(const std::string &) override { opened=true; }
  void close() override { opened=false;replies.clear(); }
  bool is_open() const override { return opened; }
  void send(std::uint16_t address,const std::vector<std::uint8_t> &data) override {
    if(data[0]==0x40) ++identities;
    if(address!=1) return;
    std::vector<std::uint8_t> answer;
    switch(data[0]) {
      case 0x40: answer={0x40,0x12,1,0,9,0x20};break;
      case 0: answer={0x82,5};break;
      case 0x31: ++positions;answer={0x31,0,0,0,0,0,0};break;
      case 0x3a: case 0x3e: answer={data[0],0};break;
      case 0xf1: answer={0xf1,1};break;
      default: return;
    }
    answer.push_back(checksum(address,answer));replies.push_back({address,answer,false,0,Clock::now()});
  }
  std::optional<Frame> receive() override {
    if(replies.empty()) return std::nullopt;
    auto answer=std::move(replies.front());replies.pop_front();return answer;
  }
};
}
TEST(CanReset, TriggerRetainsDiscoveryAndNeverScansOnSuccessOrRestartFailure) {
  rclcpp::init(0,nullptr);
  for(int result:{0,1}) {
    auto transport=std::make_unique<MemoryCan>();auto *bus=transport.get();
    auto runtime=std::make_shared<BusRuntime>(BusConfig{},std::move(transport));runtime->start(true);
    const auto limit=Clock::now()+1s;
    while(Clock::now()<limit && (runtime->snapshot().scanning || !runtime->snapshot().devices[1].position.valid)) std::this_thread::sleep_for(5ms);
    EXPECT_TRUE(runtime->snapshot().devices[1].position.valid);
    const auto queries=bus->identities.load();
    std::promise<int> completion;auto ready=completion.get_future().share();
    auto node=std::make_shared<robot_can_diagnostics::DiagnosticsNode>(runtime,BusConfig{},[ready] {
      return ready.wait_for(2s)==std::future_status::ready ? ready.get() : -1;
    });
    auto client_node=std::make_shared<rclcpp::Node>("reset_test_client");
    auto client=client_node->create_client<std_srvs::srv::Trigger>("/robot/can/reset");
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);executor.add_node(client_node);
    EXPECT_TRUE(client->wait_for_service(1s));
    auto response=client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    EXPECT_EQ(executor.spin_until_future_complete(response,1s),rclcpp::FutureReturnCode::SUCCESS);
    if(response.wait_for(0s)==std::future_status::ready) EXPECT_TRUE(response.get()->success);
    auto pending=runtime->snapshot();EXPECT_FALSE(pending.open);EXPECT_TRUE(pending.devices[1].detected);
    EXPECT_FALSE(pending.devices[1].position.valid);EXPECT_FALSE(pending.devices[1].enable.valid);
    completion.set_value(result);
    const auto until=Clock::now()+350ms;
    while(Clock::now()<until) { executor.spin_some();std::this_thread::sleep_for(5ms); }
    const auto after=runtime->snapshot();EXPECT_EQ(after.open,result==0);EXPECT_FALSE(after.scanning);
    EXPECT_TRUE(after.devices[1].detected);EXPECT_EQ(bus->identities.load(),queries);
    EXPECT_EQ(after.devices[1].position.valid,result==0);
    runtime->shutdown();executor.remove_node(node);executor.remove_node(client_node);
  }
  rclcpp::shutdown();
}
TEST(ShaftTrace, BatchesDrainEveryAcceptedPositionOutsideIoAndRetainOrder) {
  rclcpp::init(0,nullptr);
  auto transport=std::make_unique<MemoryCan>();auto *bus=transport.get();
  auto runtime=std::make_shared<BusRuntime>(BusConfig{},std::move(transport));runtime->start(true);
  auto node=std::make_shared<robot_can_diagnostics::DiagnosticsNode>(runtime,BusConfig{},[]{return 0;});
  auto receiver=std::make_shared<rclcpp::Node>("shaft_trace_test_receiver");
  std::vector<robot_control_interfaces::msg::ShaftPositionSample> points;unsigned batches=0;
  auto subscription=receiver->create_subscription<robot_control_interfaces::msg::ShaftTraceBatch>(
    "/robot/diagnostics/shaft_trace",rclcpp::QoS(20).best_effort(),[&](const robot_control_interfaces::msg::ShaftTraceBatch &message){
      ++batches;EXPECT_EQ(message.dropped_total,0u);
      for(const auto &p:message.samples) points.push_back(p);
    });
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);executor.add_node(receiver);
  auto until=Clock::now()+900ms;
  while(Clock::now()<until) {executor.spin_some();std::this_thread::sleep_for(2ms);}
  ASSERT_GT(points.size(),5u);EXPECT_GE(batches,10u);
  for(unsigned i=1;i<points.size();++i) {
    EXPECT_EQ(points[i].sequence,points[i-1].sequence+1);EXPECT_EQ(points[i].segment,points[0].segment);
    EXPECT_GT(points[i].monotonic_ns,points[i-1].monotonic_ns);EXPECT_EQ(points[i].axis,1);
    EXPECT_EQ(points[i].ticks,0);
  }
  // Brak pracy executora nie blokuje IO. Kolejny timer odbiera zaległą paczkę.
  const auto before=bus->positions.load();std::this_thread::sleep_for(180ms);
  EXPECT_GT(bus->positions.load(),before);const auto previous=points.back().sequence;
  until=Clock::now()+150ms;
  while(Clock::now()<until) {executor.spin_some();std::this_thread::sleep_for(2ms);}
  EXPECT_GT(points.back().sequence,previous+3);
  for(unsigned i=1;i<points.size();++i) EXPECT_EQ(points[i].sequence,points[i-1].sequence+1);
  runtime->shutdown();executor.remove_node(node);executor.remove_node(receiver);rclcpp::shutdown();
}
