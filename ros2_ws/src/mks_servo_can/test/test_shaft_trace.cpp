#include <gtest/gtest.h>
#include <thread>
#include "mks_servo_can/shaft_trace.hpp"
using namespace mks_servo_can;

TEST(ShaftTraceQueue, FullQueueDropsNewSamplesAndRetainsOrderAcrossWraps) {
  auto queue=std::make_unique<ShaftTraceQueue>();
  for(unsigned pass=0;pass<4;++pass) {
    for(unsigned i=0;i<4096;++i) EXPECT_TRUE(queue->push({i+1,42,7,9,-123,1,6}));
    EXPECT_FALSE(queue->push({}));
    std::array<ShaftPositionSample,256> out;
    for(unsigned block=0;block<16;++block) {
      ASSERT_EQ(queue->drain(out.data(),out.size()),256u);
      for(unsigned i=0;i<256;++i) {
        EXPECT_EQ(out[i].sequence,block*256+i+1);EXPECT_EQ(out[i].ticks,-123);
        EXPECT_EQ(out[i].axis,6);EXPECT_EQ(out[i].segment,42u);
      }
    }
    EXPECT_EQ(queue->drain(out.data(),out.size()),0u);
  }
  EXPECT_EQ(queue->dropped(),4u);
}
TEST(ShaftTraceQueue, ConcurrentProducerAndConsumerPreserveEveryAcceptedSample) {
  auto queue=std::make_unique<ShaftTraceQueue>();
  std::thread producer([&] {for(unsigned i=1;i<=100000;++i) {while(!queue->push({i,1,i,i,-static_cast<std::int64_t>(i),0,1})) std::this_thread::yield();}});
  std::array<ShaftPositionSample,256> out;unsigned expected=1;
  while(expected<=100000) {
    const auto n=queue->drain(out.data(),out.size());
    for(unsigned i=0;i<n;++i,++expected) {EXPECT_EQ(out[i].sequence,expected);EXPECT_EQ(out[i].ticks,-static_cast<std::int64_t>(expected));}
    if(!n) std::this_thread::yield();
  }
  producer.join();
}
