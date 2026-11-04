#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "streaming_fixture.hpp"
using namespace streaming_test;
namespace {
BusConfig tcp_config() {auto c=config();c.tcp_jog_enabled=true;return c;}
std::uint64_t begin(Rig &rig,std::uint64_t sequence=1) {
  rig.bus->update_motion_model("model");return rig.bus->begin_tcp_jog("session","bench","model",sequence,rig.now,rig.now);
}
// Generator testowy: ruch wszystkich osi ze stałą prędkością od pozycji 0, punkty co 10 ms jak TcpJog.
struct Feeder {
  double velocity{.05};unsigned next{};std::uint64_t keepalive{100};
  ProgramPoint point(unsigned k) const {
    ProgramPoint p;p.seconds=k*.01;
    if(k) {p.positions.fill(velocity*(p.seconds-.005));p.velocities.fill(velocity);}
    return p;
  }
  bool append(Rig &rig,std::uint64_t generation,double horizon=.2,bool closed=false) {
    const auto state=rig.bus->tcp_trajectory(rig.now);std::vector<ProgramPoint> points;
    while(next*.01<=state.elapsed+horizon) points.push_back(point(next++));
    return points.empty() || rig.bus->append_tcp_points("session",generation,points,closed,rig.now);
  }
  // Panel podtrzymuje sesję co 50 ms; tu co krok 10 ms, razem z dopełnieniem kolejki.
  void run(Rig &rig,std::uint64_t generation,unsigned steps) {
    for(unsigned i=0;i<steps && !rig.bus->control_state().fault;++i) {
      ASSERT_TRUE(append(rig,generation));
      ASSERT_TRUE(rig.bus->tcp_jog_control("session",generation,++keepalive,rig.now,rig.now));rig.run(10);
    }
  }
};
double shaft_ticks(const AxisConfig &a,double q) {return a.direction*(q-a.offset)*a.gear_ratio*16384/(2*std::acos(-1.));}
void settle(Rig &rig,std::uint64_t generation,unsigned limit_ms=3000) {
  for(unsigned i=0;i<limit_ms/10 && !rig.bus->tcp_trajectory(rig.now).done && !rig.bus->control_state().fault;++i) rig.run(10);
  ASSERT_FALSE(rig.bus->control_state().fault) << rig.bus->snapshot().message;
  ASSERT_TRUE(rig.bus->tcp_trajectory(rig.now).done);
  ASSERT_TRUE(rig.bus->finish_tcp_jog("session",generation));
  EXPECT_FALSE(rig.bus->control_state().active);
}
}
TEST(TcpJogOwner, DisabledMissingAxisCalibrationAndWrongModelRejectStart) {
  Rig disabled;disabled.ready();EXPECT_EQ(begin(disabled),0u);
  Rig missing(tcp_config());missing.port->absent=6;missing.ready();EXPECT_EQ(begin(missing),0u);
  Rig rig(tcp_config());rig.ready();rig.bus->update_motion_model("model");
  EXPECT_EQ(rig.bus->begin_tcp_jog("session","other","model",1,rig.now,rig.now),0u);
  EXPECT_EQ(rig.bus->begin_tcp_jog("session","bench","other",1,rig.now,rig.now),0u);
  EXPECT_EQ(rig.bus->begin_tcp_jog("session","bench","model",1,rig.now-100ms,rig.now),0u);
}
TEST(TcpJogOwner, KeepaliveLossStopsOpenQueueEvenWhenGeneratorKeepsAppending) {
  Rig rig(tcp_config());rig.ready();const auto generation=begin(rig);ASSERT_NE(generation,0u);Feeder feed;
  for(unsigned i=0;i<25 && !rig.bus->control_state().fault;++i) {feed.append(rig,generation);rig.run(10);}
  EXPECT_TRUE(rig.bus->control_state().fault);EXPECT_GT(rig.port->counts[0xf7],0u);
  EXPECT_NE(rig.bus->snapshot().message.find("KEEPALIVE"),std::string::npos) << rig.bus->snapshot().message;
  EXPECT_FALSE(feed.append(rig,generation));
}
TEST(TcpJogOwner, QueueRejectsForeignSessionsAndInvalidPoints) {
  Rig rig(tcp_config());rig.ready();const auto generation=begin(rig);ASSERT_NE(generation,0u);
  EXPECT_FALSE(rig.bus->activate(Source::trajectory,rig.now));
  rig.bus->renew_source_lease(Source::trajectory,rig.now);
  EXPECT_EQ(rig.bus->control_state().source,Source::tcp_jog);
  CommandBatch old;old.source=Source::tcp_jog;old.control_generation=generation;old.sequence=1;old.velocities_valid=true;
  old.configuration_generation=rig.bus->control_state().configuration_generation;
  EXPECT_FALSE(rig.bus->submit(old)) << "JOG TCP nie przyjmuje pojedynczych paczek";
  EXPECT_FALSE(rig.bus->tcp_jog_control("other",generation,2,rig.now,rig.now));
  EXPECT_FALSE(rig.bus->tcp_jog_control("session",generation+1,2,rig.now,rig.now));
  EXPECT_TRUE(rig.bus->tcp_jog_control("session",generation,2,rig.now,rig.now));
  EXPECT_FALSE(rig.bus->tcp_jog_control("session",generation,2,rig.now,rig.now));
  Feeder feed;const auto start=feed.point(0),next=feed.point(1);
  EXPECT_FALSE(rig.bus->append_tcp_points("other",generation,{start},false,rig.now));
  EXPECT_FALSE(rig.bus->append_tcp_points("session",generation+1,{start},false,rig.now));
  EXPECT_FALSE(rig.bus->append_tcp_points("session",generation,{next},false,rig.now)) << "pierwszy punkt t=0";
  auto moving=start;moving.velocities[2]=.1;
  EXPECT_FALSE(rig.bus->append_tcp_points("session",generation,{moving},false,rig.now)) << "start z postoju";
  auto beyond=next;beyond.positions[1]=2.5;
  EXPECT_FALSE(rig.bus->append_tcp_points("session",generation,{start,beyond},false,rig.now)) << "limit osi";
  auto gap=next;gap.seconds=.06;
  EXPECT_FALSE(rig.bus->append_tcp_points("session",generation,{start,gap},false,rig.now)) << "luka > 50 ms";
  auto fast=next;fast.velocities[0]=3;
  EXPECT_FALSE(rig.bus->append_tcp_points("session",generation,{start,fast},false,rig.now)) << "limit prędkości";
  EXPECT_TRUE(rig.bus->append_tcp_points("session",generation,{start,next},false,rig.now));
  EXPECT_FALSE(rig.bus->cut_tcp_jog("other",generation,"x",rig.now));
  EXPECT_FALSE(rig.bus->finish_tcp_jog("session",generation)) << "brak potwierdzonego dolotu";
  ASSERT_TRUE(rig.bus->cut_tcp_jog("session",generation,"STOP",rig.now));
  EXPECT_FALSE(rig.bus->append_tcp_points("session",generation,{feed.point(2)},false,rig.now)) << "kolejka ucięta";
  settle(rig,generation);
  const auto again=begin(rig,3);EXPECT_GT(again,generation);
}
TEST(TcpJogOwner, MeasuredMechanicalLimitAndModelChangeStopBeforeNewCommands) {
  for(bool model_change:{false,true}) {
    Rig rig(tcp_config());rig.ready();const auto generation=begin(rig);ASSERT_NE(generation,0u);
    if(model_change) rig.bus->update_motion_model("different");
    else {rig.port->frozen[1]=true;rig.port->positions[1]=static_cast<std::int64_t>(2.2*10*16384/(2*std::acos(-1.)));}
    Feeder feed;feed.run(rig,generation,10);
    EXPECT_TRUE(rig.bus->control_state().fault);
  }
}
TEST(TcpJogOwner, F5FollowsSessionGridAndPrecedesSlotQueries) {
  Rig rig(tcp_config());rig.ready();const auto started=rig.now;const auto generation=begin(rig);ASSERT_NE(generation,0u);
  rig.run(7);const auto first=rig.port->events.size();Feeder feed;feed.run(rig,generation,30);
  ASSERT_FALSE(rig.bus->control_state().fault) << rig.bus->snapshot().message;
  unsigned frames=0;
  for(auto i=first;i<rig.port->events.size();++i) {
    const auto &e=rig.port->events[i];if(e.data[0]!=0xf5) continue;++frames;
    EXPECT_LT((e.stamp-started)%kStreamingCycle,kTcpSendWindow) << "ms=" << (e.stamp-started).count()/1e6;
    // W chwili F5 żadna oś nie czeka na odpowiedź: ostatnie zapytanie ma co najmniej 1 ms zapasu.
    for(auto k=i;k-->first;) {
      const auto &q=rig.port->events[k];if(q.axis!=e.axis) continue;
      if(q.data[0]==0x31 || q.data[0]==0x3a || q.data[0]==0x3e || q.data[0]==0x32 || q.data[0]==0xf1) {EXPECT_GE(e.stamp-q.stamp,1ms);break;}
      if(q.data[0]==0xf5) break;
    }
  }
  EXPECT_GE(frames,26u*6);
}
TEST(TcpJogOwner, SinglePositionLossKeepsMotionAndSeriesStillStops) {
  Rig rig(tcp_config());rig.ready();const auto generation=begin(rig);ASSERT_NE(generation,0u);
  rig.run(3);Feeder feed;feed.velocity=.01;feed.run(rig,generation,10);
  for(unsigned axis=1;axis<=6;++axis) {rig.port->drop_position[axis]=2;feed.run(rig,generation,8);}
  EXPECT_FALSE(rig.bus->control_state().fault) << rig.bus->snapshot().message;
  for(unsigned j=0;j<6;++j) EXPECT_TRUE(rig.bus->control_state().positions[j].fresh(rig.now,40ms));
  rig.port->drop_position[2]=10;feed.run(rig,generation,10);
  EXPECT_TRUE(rig.bus->control_state().fault);
  EXPECT_NE(rig.bus->snapshot().message.find("J2 (CAN ID 2): pozycja (0x31)"),std::string::npos) << rig.bus->snapshot().message;
}
TEST(TcpJogOwner, MissingDiagnosticReplyDoesNotDelayPosition) {
  Rig rig(tcp_config());rig.ready();const auto generation=begin(rig);ASSERT_NE(generation,0u);
  rig.port->missing_speed=true;rig.run(5);Feeder feed;feed.velocity=.01;feed.run(rig,generation,40);
  EXPECT_FALSE(rig.bus->control_state().fault) << rig.bus->snapshot().message;
  for(unsigned j=0;j<6;++j) EXPECT_TRUE(rig.bus->control_state().positions[j].fresh(rig.now,25ms));
}
TEST(TcpJogOwner, CutEndsQueueAtLookaheadPointWithoutRampAndConfirmsLikeLin) {
  Rig rig(tcp_config());rig.ready();const auto generation=begin(rig);ASSERT_NE(generation,0u);
  Feeder feed;feed.run(rig,generation,50);ASSERT_FALSE(rig.bus->control_state().fault) << rig.bus->snapshot().message;
  const auto before=rig.bus->tcp_trajectory(rig.now);ASSERT_TRUE(before.started);
  EXPECT_GE(before.buffered_until-before.elapsed,.17) << "generator utrzymuje ~200 ms kolejki (dopełnia co 10 ms)";
  ASSERT_TRUE(rig.bus->cut_tcp_jog("session",generation,"STOP panelu",rig.now));
  const auto cut=rig.bus->tcp_trajectory(rig.now);ASSERT_TRUE(cut.cut);
  EXPECT_NEAR(cut.end,before.elapsed+.085,1e-9) << "koniec = cel, do którego napęd już jedzie";
  EXPECT_DOUBLE_EQ(cut.buffered_until,cut.end);
  const double target=feed.velocity*(cut.end-.005);
  // Do końca kolejki referencja biegnie dalej z pełną prędkością; potem brak nowych celów poza dolotem.
  rig.run(100);const auto first=rig.port->events.size();
  settle(rig,generation);
  for(auto i=first;i<rig.port->events.size();++i) EXPECT_NE(rig.port->events[i].data[0],0xf5) << "napęd stał w tolerancji: bez korekt";
  const auto config=rig.bus->config();
  for(unsigned j=0;j<6;++j) EXPECT_NEAR(static_cast<double>(rig.port->positions[j+1]),shaft_ticks(config.axes[j],target),2.) << "J" << j+1;
}
TEST(TcpJogOwner, GeneratorUnderrunCutsAtBufferEndWithoutFault) {
  Rig rig(tcp_config());rig.ready();const auto generation=begin(rig);ASSERT_NE(generation,0u);
  Feeder feed;feed.run(rig,generation,20);const auto last=feed.next-1;
  for(unsigned i=0;i<40;++i) {ASSERT_TRUE(rig.bus->tcp_jog_control("session",generation,++feed.keepalive,rig.now,rig.now));rig.run(10);}
  const auto state=rig.bus->tcp_trajectory(rig.now);
  EXPECT_TRUE(state.cut);EXPECT_DOUBLE_EQ(state.end,last*.01);
  EXPECT_NE(state.reason.find("Brak danych generatora"),std::string::npos) << state.reason;
  settle(rig,generation);
}
TEST(TcpJogOwner, ClosedQueueEndsAtLastGeneratorPoint) {
  Rig rig(tcp_config());rig.ready();const auto generation=begin(rig);ASSERT_NE(generation,0u);
  std::vector<ProgramPoint> points(11);
  for(unsigned k=0;k<points.size();++k) {points[k].seconds=k*.01;points[k].positions.fill(.0001*k);}
  ASSERT_TRUE(rig.bus->append_tcp_points("session",generation,points,true,rig.now));
  for(unsigned i=0;i<30;++i) {ASSERT_TRUE(rig.bus->tcp_jog_control("session",generation,i+10,rig.now,rig.now));rig.run(10);}
  const auto state=rig.bus->tcp_trajectory(rig.now);
  EXPECT_TRUE(state.cut);EXPECT_DOUBLE_EQ(state.end,.1);EXPECT_EQ(state.reason,"Koniec drogi JOG TCP");
  settle(rig,generation);
}
TEST(TcpJogOwner, FinalApproachCorrectsDriveOffsetLikePtp) {
  Rig rig(tcp_config());rig.ready();
  rig.port->drive_offset[6]=360;  // ≈0,79° przegubu: poza tolerancją 0,5°, w limicie śledzenia 1,25°
  const auto generation=begin(rig);ASSERT_NE(generation,0u);
  std::vector<ProgramPoint> hold(6);for(unsigned k=0;k<hold.size();++k) hold[k].seconds=k*.01;
  ASSERT_TRUE(rig.bus->append_tcp_points("session",generation,hold,true,rig.now));
  settle(rig,generation);
  EXPECT_LE(std::abs(static_cast<double>(rig.port->positions[6]))*2*std::acos(-1.)/(10*16384),kProgramPositionTolerance);
  for(unsigned j=0;j<5;++j) EXPECT_EQ(rig.port->positions[j+1],0) << "J" << j+1;
}
TEST(TcpJogOwner, ShaftTraceRecordsPlannedReferenceInShaftTicks) {
  Rig rig(tcp_config());rig.ready();const auto generation=begin(rig);ASSERT_NE(generation,0u);
  std::array<ShaftPositionSample,4096> out;rig.bus->drain_shaft_trace(out.data(),out.size());
  const auto config=rig.bus->config();
  std::vector<ProgramPoint> hold(40);for(unsigned k=0;k<hold.size();++k) {hold[k].seconds=k*.01;hold[k].positions.fill(.001);}
  hold[0].positions.fill(0);
  ASSERT_TRUE(rig.bus->append_tcp_points("session",generation,hold,true,rig.now));
  for(unsigned i=0;i<30;++i) {ASSERT_TRUE(rig.bus->tcp_jog_control("session",generation,i+10,rig.now,rig.now));rig.run(10);}
  ASSERT_FALSE(rig.bus->control_state().fault) << rig.bus->snapshot().message;
  const auto count=rig.bus->drain_shaft_trace(out.data(),out.size());
  std::array<unsigned,7> references{},measured{};std::array<std::uint64_t,7> previous{};
  for(std::size_t i=0;i<count;++i) {
    const auto &p=out[i];
    if(p.flags&(kShaftCommandFlag|kShaftRpmFlag)) continue;
    if(!(p.flags&kShaftReferenceFlag)) {++measured[p.axis];continue;}
    if(previous[p.axis]) {
      // Po pierwszych 20 ms (Hermite z postoju) cel planowany = .001, ta sama skala impulsów co pomiar 0x31.
      if(references[p.axis]>=3) EXPECT_EQ(p.ticks,std::llround(shaft_ticks(config.axes[p.axis-1],.001)));
      EXPECT_GE(p.monotonic_ns-previous[p.axis],9'000'000u);
    }
    previous[p.axis]=p.monotonic_ns;++references[p.axis];
  }
  for(unsigned axis=1;axis<=6;++axis) {EXPECT_GE(references[axis],25u) << axis;EXPECT_LE(references[axis],31u) << axis;EXPECT_GT(measured[axis],0u) << axis;}
}
TEST(TcpJogOwner, ShaftTraceRecordsSentF5TargetAndRpm) {
  Rig rig(tcp_config());rig.ready();const auto generation=begin(rig);ASSERT_NE(generation,0u);
  std::array<ShaftPositionSample,4096> out;rig.bus->drain_shaft_trace(out.data(),out.size());
  Feeder feed;feed.run(rig,generation,30);
  ASSERT_FALSE(rig.bus->control_state().fault) << rig.bus->snapshot().message;
  const auto count=rig.bus->drain_shaft_trace(out.data(),out.size());
  std::array<unsigned,7> targets{},speeds{};std::array<std::int64_t,7> last{};
  for(std::size_t i=0;i<count;++i) {
    const auto &p=out[i];
    EXPECT_FALSE((p.flags&kShaftCommandFlag) && (p.flags&kShaftRpmFlag));
    if(p.flags&kShaftCommandFlag) {
      // Ruch ze stałą prędkością: cel F5 (z wyprzedzeniem) nie cofa się.
      if(targets[p.axis]) EXPECT_GE(p.ticks,last[p.axis]);
      last[p.axis]=p.ticks;++targets[p.axis];
    }
    if(p.flags&kShaftRpmFlag) ++speeds[p.axis];
  }
  for(unsigned axis=1;axis<=6;++axis) {EXPECT_GE(targets[axis],25u) << axis;EXPECT_EQ(targets[axis],speeds[axis]) << axis;}
}
TEST(DriveParameters, IdleReadsCurrentPidAndAngleErrorWithoutCountingSilenceAsLoss) {
  for(unsigned variant=0;variant<3;++variant) {
    Rig rig(tcp_config());rig.port->angle_error[4]=-284;  // −2° wału (51200 = 360°)
    rig.port->pid_without_sub=variant==1;rig.port->silent_pid=variant==2;
    rig.ready();rig.run(300);
    for(unsigned axis=1;axis<=6;++axis) EXPECT_FALSE(rig.bus->snapshot().devices[axis].work_current_ma.valid) << "bez żądania brak odczytów";
    ASSERT_TRUE(rig.bus->request_drive_parameters());EXPECT_TRUE(rig.bus->drive_parameters_pending());
    rig.run(800);EXPECT_FALSE(rig.bus->drive_parameters_pending()) << "żądanie zakończone";
    const auto snapshot=rig.bus->snapshot();
    for(unsigned axis=1;axis<=6;++axis) {
      const auto &d=snapshot.devices[axis];
      EXPECT_TRUE(d.work_current_ma.valid) << axis;EXPECT_EQ(d.work_current_ma.value,1600u) << axis;
      EXPECT_EQ(d.pid_kp_ki.valid,variant!=2) << axis;EXPECT_EQ(d.pid_kd_kv.valid,variant!=2) << axis;
      if(variant!=2) {EXPECT_EQ(d.pid_kp_ki.value[0],0xdc);EXPECT_EQ(d.pid_kp_ki.value[1],0x64);EXPECT_EQ(d.pid_kd_kv.value[0],0x10e);EXPECT_EQ(d.pid_kd_kv.value[1],0x140);}
      EXPECT_TRUE(d.angle_error.valid) << axis;
      EXPECT_LT(d.consecutive_timeouts,3u) << "brak odczytu PID nie oznacza utraty osi";
    }
    EXPECT_EQ(snapshot.devices[4].angle_error.value,-284);
    unsigned pid_reads=0;
    for(const auto &e:rig.port->events) if(e.axis==2 && e.data.size()==4 && e.data[0]==0 && e.data[1]==0x96) ++pid_reads;
    EXPECT_LE(pid_reads,6u) << "najwyżej 3 próby na parametr";
    EXPECT_FALSE(snapshot.fault) << snapshot.message;
  }
}
TEST(DriveParameters, NoDiagnosticParameterReadsDuringMotion) {
  Rig rig(tcp_config());rig.ready();ASSERT_TRUE(rig.bus->request_drive_parameters());
  const auto generation=begin(rig);ASSERT_NE(generation,0u);EXPECT_FALSE(rig.bus->drive_parameters_pending()) << "start ruchu anuluje żądanie";
  const auto first=rig.port->events.size();Feeder feed;feed.run(rig,generation,30);
  ASSERT_FALSE(rig.bus->control_state().fault) << rig.bus->snapshot().message;
  for(auto i=first;i<rig.port->events.size();++i) {
    const auto &e=rig.port->events[i];
    EXPECT_FALSE(e.data[0]==0x39 || (e.data.size()>=2 && e.data[0]==0 && (e.data[1]==0x83 || e.data[1]==0x96))) << int(e.data[0]);
  }
}
