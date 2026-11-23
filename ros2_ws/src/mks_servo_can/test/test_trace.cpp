#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <unistd.h>
#include "mks_servo_can/trace.hpp"
using namespace mks_servo_can;
TEST(TraceRecorder, RotationIsBoundedAndEventsCarryBothClocks) {
  char name[] = "/tmp/robot-trace-test-XXXXXX";
  const auto directory = std::string(::mkdtemp(name));
  {
    auto recorder = std::make_unique<TraceRecorder>(directory, 2048);
    std::uint64_t accepted = 0;
    for (unsigned i = 0; i < 10000; ++i) {
      TraceEvent e; e.event = "test"; e.axis = 2; e.command = 0x31;
      e.detail[0] = '"'; e.detail[1] = '\n';
      if (recorder->submit(e)) ++accepted;
    }
    EXPECT_GT(accepted, 0u); EXPECT_EQ(accepted + recorder->dropped(), 10000u);
  }
  unsigned files = 0, lines = 0;
  for (const auto &entry : std::filesystem::directory_iterator(directory)) {
    ++files; EXPECT_LE(entry.file_size(), 2048u);
    std::ifstream input(entry.path()); std::string line;
    while (std::getline(input, line)) {
      ++lines; EXPECT_NE(line.find("\"mono_ns\":"), std::string::npos);
      EXPECT_NE(line.find("\"utc_ns\":"), std::string::npos);
      if (line.find("\"event\":\"test\"") != std::string::npos)
        EXPECT_NE(line.find("\\\"\\u000a"), std::string::npos);
      EXPECT_EQ(line.front(), '{'); EXPECT_EQ(line.back(), '}');
    }
  }
  EXPECT_LE(files, 4u); EXPECT_GT(lines, 0u);
  std::filesystem::remove_all(directory);
}
TEST(TraceRecorder, InvalidDirectoryFailsWithoutStartingWriter) {
  EXPECT_THROW(TraceRecorder("/dev/null/unavailable"), std::exception);
}
TEST(TraceRecorder, FaultEventCreatesSnapshotTriggerAndRecordingContinues) {
  char name[] = "/tmp/robot-trace-fault-XXXXXX";
  const auto directory = std::string(::mkdtemp(name));
  {
    auto recorder = std::make_unique<TraceRecorder>(directory, 4096, 3);
    TraceEvent fault; fault.event = "bus_fault";
    const std::string detail = "J5: uchyb \"testowy\"";
    std::copy(detail.begin(), detail.end(), fault.detail.begin());
    TraceEvent normal; normal.event = "request_timeout";
    EXPECT_TRUE(recorder->submit(normal));
    EXPECT_TRUE(recorder->submit(fault));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    // Bez zamrożenia: kolejne zdarzenia nadal są przyjmowane.
    TraceEvent after; after.event = "after_fault";
    EXPECT_TRUE(recorder->submit(after));
  }
  unsigned triggers = 0; std::string trigger;
  for (const auto &entry : std::filesystem::directory_iterator(directory + "/faults")) {
    ++triggers; EXPECT_EQ(entry.path().extension(), ".json");
    std::ifstream file(entry.path()); trigger.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
  }
  EXPECT_EQ(triggers, 1u) << "tylko zdarzenie awarii tworzy wyzwalacz";
  EXPECT_NE(trigger.find("\"event\":\"bus_fault\""), std::string::npos);
  EXPECT_NE(trigger.find("\\\"testowy\\\""), std::string::npos);
  std::ifstream log(directory + "/can-0.jsonl"); std::string text((std::istreambuf_iterator<char>(log)), std::istreambuf_iterator<char>());
  EXPECT_NE(text.find("after_fault"), std::string::npos);
  EXPECT_TRUE(is_fault_trace_event("manual_fault")); EXPECT_FALSE(is_fault_trace_event("manual_stop"));
  EXPECT_FALSE(is_fault_trace_event(nullptr));
  std::filesystem::remove_all(directory);
}
TEST(TraceRecorder, RingUsesConfiguredSegmentCount) {
  char name[] = "/tmp/robot-trace-ring-XXXXXX";
  const auto directory = std::string(::mkdtemp(name));
  {
    auto recorder = std::make_unique<TraceRecorder>(directory, 2048, 6);
    for (unsigned i = 0; i < 3000; ++i) {
      TraceEvent e; e.event = "ring";
      while (!recorder->submit(e)) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  unsigned files = 0;
  for (const auto &entry : std::filesystem::directory_iterator(directory)) { ++files; EXPECT_LE(entry.file_size(), 2048u); }
  EXPECT_EQ(files, 6u);
  EXPECT_THROW(TraceRecorder(directory, 2048, 1), std::invalid_argument);
  std::filesystem::remove_all(directory);
}
TEST(TraceRecorder, ConcurrentProducersDoNotLoseEntriesDuringWriterDrain) {
  char name[] = "/tmp/robot-trace-concurrent-XXXXXX";
  const auto directory = std::string(::mkdtemp(name));
  {
    auto recorder = std::make_unique<TraceRecorder>(directory);
    auto producer = [&recorder](unsigned axis) {
      for (unsigned i = 0; i < 200; ++i) { TraceEvent e; e.event = "concurrent"; e.axis = axis; recorder->submit(e); }
    };
    std::thread first(producer, 1), second(producer, 2); first.join(); second.join();
    EXPECT_EQ(recorder->dropped(), 0u);
  }
  std::ifstream file(directory + "/can-0.jsonl"); std::string line; unsigned count = 0;
  while (std::getline(file, line)) if (line.find("\"event\":\"concurrent\"") != std::string::npos) ++count;
  EXPECT_EQ(count, 400u); std::filesystem::remove_all(directory);
}
