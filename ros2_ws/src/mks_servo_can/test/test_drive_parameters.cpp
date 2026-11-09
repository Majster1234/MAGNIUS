#include <gtest/gtest.h>
#include "streaming_fixture.hpp"
#include "mks_servo_can/drive_parameters.hpp"
using namespace mks_servo_can;
using namespace std::chrono_literals;
using namespace streaming_test;

TEST(DriveParameterCodec, ReadWriteRequestsAndReadOnlyCodes) {
  const auto *mode=find_drive_parameter(0x82,-1);ASSERT_TRUE(mode);
  EXPECT_EQ(drive_parameter_read_request(1,*mode),(std::vector<std::uint8_t>{0x00,0x82,0x83}));
  const auto *kd=find_drive_parameter(0x96,1);ASSERT_TRUE(kd);
  EXPECT_EQ(drive_parameter_read_request(1,*kd),(std::vector<std::uint8_t>{0x00,0x96,0x01,0x98}));
  EXPECT_EQ(drive_parameter_write_request(1,*mode,{5}),(std::vector<std::uint8_t>{0x82,5,0x88}));
  EXPECT_THROW(drive_parameter_write_request(1,*mode,{5,0}),std::invalid_argument);
  EXPECT_THROW(drive_parameter_write_request(1,*kd,{0,1,0,1,0}),std::invalid_argument) << "zła podkomenda";
  for(std::uint8_t code:{0x8a,0x8c,0x4a,0x0f,0x90,0x94,0x9a,0x9e,0xff}) {
    const auto *spec=find_drive_parameter(code,-1);ASSERT_TRUE(spec) << int(code);
    EXPECT_FALSE(spec->writable) << int(code);
    EXPECT_THROW(drive_parameter_write_request(1,*spec,std::vector<std::uint8_t>(spec->length,0)),std::invalid_argument);
  }
  for(std::uint8_t code:{0x8b,0x80,0x3f,0x41}) EXPECT_EQ(find_drive_parameter(code,-1),nullptr) << "adres/kalibracja/reset poza katalogiem";
}
TEST(DriveParameterCodec, ParseNormalizesPidUnsupportedAndCurrentPadding) {
  const auto *kp=find_drive_parameter(0x96,0);
  auto frame=[](std::uint16_t address,std::vector<std::uint8_t> d) {unsigned s=address;for(auto b:d) s+=b;d.push_back(static_cast<std::uint8_t>(s));return d;};
  auto with_sub=parse_drive_parameter(3,*kp,frame(3,{0x96,0,0,0xdc,0,0x64}));
  auto without=parse_drive_parameter(3,*kp,frame(3,{0x96,0,0xdc,0,0x64}));
  ASSERT_TRUE(with_sub && without);EXPECT_EQ(with_sub->value,without->value);EXPECT_EQ(with_sub->value.size(),5u);
  const auto none=parse_drive_parameter(3,*find_drive_parameter(0x9d,-1),frame(3,{0x9d,0xff,0xff}));
  ASSERT_TRUE(none);EXPECT_FALSE(none->supported);
  const auto current=parse_drive_parameter(2,*find_drive_parameter(0x83,-1),frame(2,{0x83,0x06,0x40,0x00}));
  ASSERT_TRUE(current);EXPECT_EQ(current->value,(std::vector<std::uint8_t>{0x06,0x40}));
  EXPECT_FALSE(parse_drive_parameter(2,*find_drive_parameter(0x83,-1),{0x83,0x06,0x40,0x00}));  // zła suma
  EXPECT_EQ(parse_drive_parameter_write(2,*find_drive_parameter(0x83,-1),frame(2,{0x83,1})),std::optional<bool>(true));
  EXPECT_EQ(parse_drive_parameter_write(2,*find_drive_parameter(0x83,-1),frame(2,{0x83,0})),std::optional<bool>(false));
}
namespace {
CanBusManager::ParameterJobResult wait_job(Rig &h,std::uint64_t id) {
  for(unsigned i=0;i<500;++i) {h.run(1);auto r=h.bus->parameter_job(id);if(r.done) return r;}
  return h.bus->parameter_job(id);
}
}
TEST(DriveParameterJobs, ReadWriteRequireIdleAndEnableOff) {
  Rig h;h.ready();std::string reason;
  h.port->params[Port::param_key(2,0x83,-1)]={0x06,0x40};
  h.port->params[Port::param_key(2,0x96,1)]={0x01,0x01,0x0e,0x01,0x40};
  auto id=h.bus->submit_parameter_job(2,0x83,-1,false,{},h.now,reason);ASSERT_TRUE(id) << reason;
  auto r=wait_job(h,id);ASSERT_TRUE(r.done && r.success) << r.message;EXPECT_EQ(r.value,(std::vector<std::uint8_t>{0x06,0x40}));
  id=h.bus->submit_parameter_job(2,0x96,1,false,{},h.now,reason);r=wait_job(h,id);
  ASSERT_TRUE(r.success) << r.message;EXPECT_EQ(r.value,(std::vector<std::uint8_t>{0x01,0x01,0x0e,0x01,0x40}));
  EXPECT_FALSE(h.bus->submit_parameter_job(2,0x83,-1,true,{0x07,0xd0},h.now,reason));
  EXPECT_NE(reason.find("ENABLE=0"),std::string::npos) << reason;
  // Wyjątek PID: zapis przy ENABLE=1 bez aktywnego sterowania.
  id=h.bus->submit_parameter_job(2,0x96,1,true,{0x01,0x01,0x20,0x01,0x40},h.now,reason);ASSERT_TRUE(id) << reason;
  r=wait_job(h,id);ASSERT_TRUE(r.success) << r.message;
  EXPECT_EQ(h.port->params[Port::param_key(2,0x96,1)],(std::vector<std::uint8_t>{0x01,0x01,0x20,0x01,0x40}));
  EXPECT_FALSE(h.bus->snapshot().fault);
  EXPECT_FALSE(h.bus->submit_parameter_job(2,0x8a,-1,true,{3},h.now,reason)) << "bitrate tylko do odczytu";
  EXPECT_FALSE(h.bus->submit_parameter_job(2,0x8b,-1,false,{},h.now,reason)) << "adres poza katalogiem";
  h.port->enabled[2]=false;h.run(400);
  id=h.bus->submit_parameter_job(2,0x83,-1,true,{0x07,0xd0},h.now,reason);ASSERT_TRUE(id) << reason;
  r=wait_job(h,id);ASSERT_TRUE(r.success) << r.message;
  EXPECT_EQ(h.port->params[Port::param_key(2,0x83,-1)],(std::vector<std::uint8_t>{0x07,0xd0}));
  id=h.bus->submit_parameter_job(2,0x83,-1,false,{},h.now,reason);r=wait_job(h,id);
  EXPECT_EQ(r.value,(std::vector<std::uint8_t>{0x07,0xd0})) << "odczyt kontrolny po zapisie";
  h.port->reject_param_write=0x9b;h.port->params[Port::param_key(2,0x9b,-1)]={4};
  id=h.bus->submit_parameter_job(2,0x9b,-1,true,{2},h.now,reason);r=wait_job(h,id);
  EXPECT_TRUE(r.done);EXPECT_FALSE(r.success);EXPECT_NE(r.message.find("odrzucił"),std::string::npos) << r.message;
  EXPECT_FALSE(h.bus->snapshot().fault) << "odrzucony zapis nie jest awarią: " << h.bus->snapshot().message;
}
TEST(DriveParameterJobs, UnsupportedReadAndBlockedDuringMotion) {
  Rig h;h.ready();std::string reason;
  h.port->unsupported_param=0x9d;
  auto id=h.bus->submit_parameter_job(1,0x9d,-1,false,{},h.now,reason);ASSERT_TRUE(id) << reason;
  auto r=wait_job(h,id);EXPECT_TRUE(r.success);EXPECT_FALSE(r.supported);
  h.execute(h.program(.2,2.));h.run(100);
  EXPECT_FALSE(h.bus->submit_parameter_job(1,0x83,-1,false,{},h.now,reason));
  EXPECT_NE(reason.find("sterowanie"),std::string::npos) << reason;
  EXPECT_FALSE(h.bus->submit_parameter_job(1,0x96,0,true,{0,0,0xdc,0,0x64},h.now,reason)) << "PID przy ENABLE=1 tylko bez ruchu";
}
