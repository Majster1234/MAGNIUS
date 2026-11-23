#include <gtest/gtest.h>
#include "mks_servo_can/socketcan.hpp"
#include "mks_servo_can/protocol.hpp"
#include <linux/can.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
namespace mks_servo_can {
struct SocketCanTestAccess {
  static void attach(SocketCan &socket,int fd) {socket.fd_=fd;socket.receiving_=true;socket.receiver_=std::thread([&socket]{socket.receive_loop();});}
};
}
using namespace mks_servo_can;
TEST(AsyncSocketCan, DedicatedReceiverPublishesOnlyValidatedPositionsWithoutConsumerIo) {
  int fd[2];ASSERT_EQ(socketpair(AF_UNIX,SOCK_DGRAM|SOCK_NONBLOCK,0,fd),0);
  SocketCan socket;SocketCanTestAccess::attach(socket,fd[0]);
  auto send=[&](std::vector<uint8_t> bytes) {can_frame frame{};frame.can_id=2;frame.can_dlc=bytes.size();std::copy(bytes.begin(),bytes.end(),frame.data);ASSERT_EQ(::write(fd[1],&frame,sizeof(frame)),static_cast<ssize_t>(sizeof(frame)));};
  send(request(2,0x31,{0xff,0xff,0xff,0xff,0xc0,0}));
  for(unsigned i=0;i<1000 && !socket.position_feedback(2).valid;++i) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  const auto sample=socket.position_feedback(2);ASSERT_TRUE(sample.valid);EXPECT_EQ(sample.pulses,-16384);EXPECT_GT(sample.timestamp_ns,0u);
  const auto raw=socket.receive();ASSERT_TRUE(raw);EXPECT_EQ(raw->data[0],0x31);
  auto invalid=request(2,0x31,{0,0,0,0,0,1});invalid.back()^=1;send(invalid);
  for(unsigned i=0;i<1000;++i) {if(socket.receive()) break;std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  EXPECT_EQ(socket.position_feedback(2).pulses,-16384);
  socket.close();::close(fd[1]);
}
TEST(AsyncSocketCan, QueuePreservesOrderAcrossThreadsAndBoundsCapacity) {
  FrameQueue queue;std::atomic<bool> done{false};constexpr unsigned count=20000;
  std::thread writer([&]{for(unsigned n=0;n<count;++n) {Frame frame;frame.kernel_utc_ns=n;while(!queue.push(frame)) std::this_thread::yield();}done=true;});
  unsigned next=0;while(!done || next<count) {if(auto frame=queue.pop()) EXPECT_EQ(frame->kernel_utc_ns,next++);else std::this_thread::yield();}
  writer.join();EXPECT_EQ(next,count);EXPECT_FALSE(queue.pop());
  for(unsigned i=0;i<4095;++i) ASSERT_TRUE(queue.push(Frame{}));
  EXPECT_FALSE(queue.push(Frame{}));
  for(unsigned i=0;i<4095;++i) ASSERT_TRUE(queue.pop());
  EXPECT_FALSE(queue.pop());
}
