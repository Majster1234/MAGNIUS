#include <gtest/gtest.h>
#include <cmath>
#include "streaming_fixture.hpp"
using namespace streaming_test;
namespace {
void wait_for(Rig &h,const std::string &state) {
  for(unsigned i=0;i<1000 && h.bus->program_status().sync_state!=state && !h.bus->snapshot().fault;++i) h.run(1);
  ASSERT_EQ(h.bus->program_status().sync_state,state)<<h.bus->program_status().message;
}
}
TEST(SynchronizedProgram, SixAxesStageThenStartTogetherOnFixedGridAndRestore) {
  Rig h;h.ready();h.synchronized();h.execute(h.program());wait_for(h,"waiting_ack");
  EXPECT_EQ(h.port->activated_sets,0u);EXPECT_EQ(h.port->positions[1],0);
  EXPECT_TRUE(h.port->pending_targets_valid[1]);h.run(1500);
  const auto s=h.bus->program_status();ASSERT_EQ(s.state,"succeeded")<<s.message;
  EXPECT_EQ(s.dispatch_mode,"synchronized");EXPECT_EQ(s.mode,"streamed");EXPECT_EQ(s.sync_state,"restored");
  EXPECT_EQ(h.port->counts[0xf5],606u);EXPECT_EQ(h.port->counts[0x4b],101u);EXPECT_EQ(s.sync_cycle,100u);
  std::vector<Time> times;for(const auto &e:h.port->events) if(e.data[0]==0x4b) times.push_back(e.stamp);
  for(unsigned i=1;i<times.size();++i) EXPECT_EQ(times[i]-times[i-1],10ms);
  for(unsigned j=1;j<=6;++j) {EXPECT_FALSE(h.port->sync[j]);EXPECT_FALSE(h.port->respond[j]);EXPECT_FALSE(h.port->pending_targets_valid[j]);}
  EXPECT_FALSE(h.bus->snapshot().fault);
}
TEST(SynchronizedProgram, RejectedAckStopsAtOnceAndPersistentMissingOrLateAckAfterLimit) {
  for(unsigned kind=0;kind<3;++kind) {
    Rig h;h.ready();h.synchronized();const bool rejected=kind==0;
    if(rejected) h.port->rejected_ack=3;else if(kind==1) h.port->missing_ack=3;else h.port->late_ack=3;
    h.execute(h.program());h.run(300);
    EXPECT_TRUE(h.bus->snapshot().fault);
    // Odrzucenie (status≠05) zatrzymuje od razu; brak ACK jest dopuszczony w dwóch kolejnych cyklach.
    // ACK spóźniony stale o 20 ms bywa przypisany późniejszemu zestawowi (brak numeru cyklu), więc tylko ≥ 2.
    if(kind==2) EXPECT_GE(h.port->counts[0x4b],2u);else EXPECT_EQ(h.port->counts[0x4b],rejected ? 0u : 2u);
    if(kind==1) EXPECT_NE(h.bus->snapshot().message.find("3 kolejnych cyklach"),std::string::npos) << h.bus->snapshot().message;
    EXPECT_GE(h.port->counts[0xf5],6u);EXPECT_GT(h.port->counts[0xf7],0u);
    const auto syncs=h.port->counts[0x4b];
    h.bus->clear_fault();h.run(200);EXPECT_TRUE(h.bus->snapshot().fault);EXPECT_EQ(h.port->counts[0x4b],syncs);
  }
}
TEST(SynchronizedProgram, OccasionalMissingAckKeepsMotionAndIsCounted) {
  Rig h;h.ready();h.synchronized();h.port->drop_ack[3]=2;
  h.execute(h.program());h.run(1600);
  const auto s=h.bus->program_status();EXPECT_EQ(s.state,"succeeded") << s.message << "; " << h.bus->snapshot().message;
  EXPECT_EQ(h.port->activated_sets,101u);EXPECT_EQ(h.bus->snapshot().statistics.sync_missed_acks,2u);
}
TEST(SynchronizedProgram, NoRequestToAxisWhileItsF5AckIsOutstanding) {
  Rig h;h.ready();h.synchronized();for(unsigned j=1;j<=6;++j) h.port->axis_ack_delay[j]=3ms;
  h.execute(h.program());h.run(1600);ASSERT_EQ(h.bus->program_status().state,"succeeded") << h.bus->program_status().message;
  unsigned checked=0;
  for(std::size_t i=0;i<h.port->events.size();++i) {
    const auto &f5=h.port->events[i];if(f5.data[0]!=0xf5 || !f5.axis) continue;++checked;
    // ACK tej osi przychodzi po 3 ms tym samym ID co host: wcześniej żadna inna ramka do tej osi.
    for(auto k=i+1;k<h.port->events.size() && h.port->events[k].stamp<f5.stamp+3ms;++k)
      EXPECT_NE(h.port->events[k].axis,f5.axis) << "opcode=" << int(h.port->events[k].data[0]) << " ms=" << (h.port->events[k].stamp-f5.stamp).count()/1e6;
  }
  EXPECT_GE(checked,600u);
}
TEST(SynchronizedProgram, ShaftTraceRecordsReferenceAndF5AtActivation) {
  Rig h;h.ready();h.synchronized();h.execute(h.program());
  std::array<ShaftPositionSample,4096> out;std::array<unsigned,7> references{},targets{},speeds{};
  for(unsigned i=0;i<16;++i) {
    h.run(100);const auto count=h.bus->drain_shaft_trace(out.data(),out.size());
    for(std::size_t k=0;k<count;++k) {
      const auto &p=out[k];
      if(p.flags&kShaftReferenceFlag) ++references[p.axis];
      if(p.flags&kShaftCommandFlag) ++targets[p.axis];
      if(p.flags&kShaftRpmFlag) ++speeds[p.axis];
    }
  }
  ASSERT_EQ(h.bus->program_status().state,"succeeded") << h.bus->program_status().message;
  for(unsigned axis=1;axis<=6;++axis) {
    EXPECT_GE(targets[axis],95u) << axis;EXPECT_EQ(targets[axis],speeds[axis]) << axis;EXPECT_GE(references[axis],95u) << axis;
  }
}
TEST(SynchronizedProgram, LostTransmitConfirmationNeverStagesNextSet) {
  Rig h;h.ready();h.synchronized();h.port->drop_sync_confirmation=true;
  h.execute(h.program());h.run(300);
  EXPECT_TRUE(h.bus->snapshot().fault);EXPECT_EQ(h.port->counts[0x4b],1u);EXPECT_GE(h.port->counts[0xf5],6u);
  EXPECT_EQ(h.bus->snapshot().statistics.committed,0u);
}
TEST(SynchronizedProgram, LateTransmitConfirmationShiftsTimeBaseInsteadOfStopping) {
  Rig h;h.ready();h.synchronized();h.port->sync_echo_delay=3ms;
  h.execute(h.program());h.run(2000);
  EXPECT_EQ(h.bus->program_status().state,"succeeded") << h.bus->program_status().message << "; " << h.bus->snapshot().message;
  EXPECT_EQ(h.port->activated_sets,101u);EXPECT_GT(h.bus->snapshot().statistics.sync_shifts,0u);
}
TEST(SynchronizedProgram, SubmitDoesNotCommitUntilOwnTransmitConfirmation) {
  Rig h;h.ready();h.synchronized();h.port->sync_echo_delay=2ms;h.execute(h.program());wait_for(h,"waiting_tx_confirmation");
  EXPECT_EQ(h.bus->snapshot().statistics.committed,0u);EXPECT_EQ(h.port->counts[0xf5],6u);
  h.run(2);EXPECT_EQ(h.bus->snapshot().statistics.committed,1u);EXPECT_EQ(h.port->counts[0xf5],12u);
}
TEST(SynchronizedProgram, TwoMillisecondSchedulerSlipKeepsOriginalEpoch) {
  Rig h;h.ready();h.synchronized();h.execute(h.program());wait_for(h,"waiting_ack");h.run(3);
  const auto due=Time{std::chrono::nanoseconds(h.bus->program_status().sync_due_monotonic_ns)};
  h.now=due+2ms;h.run(1);EXPECT_EQ(h.port->counts[0x4b],1u);h.run(30);
  EXPECT_FALSE(h.bus->snapshot().fault)<<h.bus->snapshot().message;
  std::vector<Time> times;for(const auto &e:h.port->events) if(e.data[0]==0x4b) times.push_back(e.stamp);
  ASSERT_GE(times.size(),2u);EXPECT_EQ(times[1]-times[0],8ms);
}
TEST(SynchronizedProgram, SlipWithinShiftLimitMovesTimeBaseWithoutCatchup) {
  Rig h;h.ready();h.synchronized();h.execute(h.program());wait_for(h,"waiting_ack");h.run(3);
  const auto due=Time{std::chrono::nanoseconds(h.bus->program_status().sync_due_monotonic_ns)};
  h.now=due+5ms;h.run(1);EXPECT_EQ(h.port->counts[0x4b],1u);h.run(30);
  EXPECT_FALSE(h.bus->snapshot().fault) << h.bus->snapshot().message;EXPECT_EQ(h.bus->snapshot().statistics.sync_shifts,1u);
  // Kolejne terminy liczone od przesuniętej osi czasu: pełne 10 ms, bez nadrabiania.
  std::vector<Time> times;for(const auto &e:h.port->events) if(e.data[0]==0x4b) times.push_back(e.stamp);
  ASSERT_GE(times.size(),3u);EXPECT_EQ(times[1]-times[0],10ms);EXPECT_EQ(times[2]-times[1],10ms);
}
TEST(SynchronizedProgram, SlipBeyondShiftLimitStopsWithoutSyncOrCatchup) {
  Rig h;h.ready();h.synchronized();h.execute(h.program());wait_for(h,"waiting_ack");h.run(3);
  const auto due=Time{std::chrono::nanoseconds(h.bus->program_status().sync_due_monotonic_ns)};
  h.now=due+11ms;h.run(100);EXPECT_TRUE(h.bus->snapshot().fault);EXPECT_EQ(h.port->counts[0x4b],0u);
}
TEST(SynchronizedProgram, StopAtPreparationAcceptedAndSubmittedPhasesBlocksFurtherSync) {
  for(unsigned phase=0;phase<3;++phase) {
    Rig h;h.ready();h.synchronized();h.execute(h.program());wait_for(h,"waiting_ack");
    if(phase==1) h.run(4);if(phase==2) wait_for(h,"waiting_tx_confirmation");
    const auto first=h.port->events.size();h.bus->stop("STOP operatora",false);const auto syncs=h.port->counts[0x4b];h.run(2500);
    EXPECT_EQ(h.port->counts[0x4b],syncs);EXPECT_TRUE(h.bus->snapshot().fault);
    for(auto it=h.port->events.begin()+first;it!=h.port->events.end();++it) if(it->data[0]==0xf5)
      EXPECT_EQ(it->data,std::vector<std::uint8_t>({0xf5,0,0,0,0,0,0,static_cast<std::uint8_t>(it->axis+0xf5)}));
    for(unsigned j=1;j<=6;++j) {
      EXPECT_TRUE(h.port->pending_targets_valid[j]);EXPECT_TRUE(h.port->pending_stops[j]);
      EXPECT_TRUE(h.port->sync[j]);EXPECT_FALSE(h.port->enabled[j]);
    }
  }
}
TEST(SynchronizedProgram, PartialWriteStopsWithoutTrigger) {
  Rig h;h.ready();h.synchronized();h.port->fail_f5=true;h.execute(h.program());h.run(300);
  EXPECT_TRUE(h.bus->snapshot().fault);EXPECT_EQ(h.port->counts[0x4b],0u);EXPECT_GE(h.port->counts[0xf5],3u);
}
TEST(SynchronizedProgram, UnsupportedFirmwareOrTransportRejectedBeforeArming) {
  for(bool firmware:{false,true}) {
    Rig h;h.port->bad_firmware=firmware;h.port->tx_confirmations=firmware;h.ready();h.synchronized();
    EXPECT_THROW(h.bus->prepare_program(h.program(),h.now),std::runtime_error);EXPECT_EQ(h.port->counts[0xf5],0u);
  }
}
TEST(SynchronizedProgram, NoDiagnosticRequestsInGuardBeforeNextPreparedSet) {
  Rig h;h.ready();h.synchronized();h.execute(h.program());h.run(1500);
  const auto first=std::find_if(h.port->events.begin(),h.port->events.end(),[](const auto &e){return e.data[0]==0xf5;});ASSERT_NE(first,h.port->events.end());
  const auto origin=first->stamp;const auto last=std::find_if(h.port->events.rbegin(),h.port->events.rend(),[](const auto &e){return e.data[0]==0xf5;});
  for(const auto &e:h.port->events) if(e.stamp>=origin && e.stamp<last->stamp &&
      (e.data[0]==0x31 || e.data[0]==0x32 || e.data[0]==0x3a || e.data[0]==0x3e || e.data[0]==0xf1))
    EXPECT_LT((e.stamp-origin)%10ms,6ms);
}
TEST(SynchronizedProgram, ReversalBetweenKnotsLimitsCommonLookahead) {
  PreparedMotionProgram p;p.participating.fill(true);p.duration=1;
  ProgramPoint a,b;a.velocities.fill(1);b.velocities.fill(-1);b.seconds=1;p.points={a,b};
  compile_program_reversals(p);ASSERT_FALSE(p.reversal_times.empty());EXPECT_NEAR(p.reversal_times[0],.5,1e-8);
  EXPECT_NEAR(program_lookahead_time(p,.45,.085),.5,1e-8);EXPECT_NEAR(program_lookahead_time(p,.51,.085),.595,1e-8);
}
TEST(SynchronizedProgram, DispatchChangePreservesCalibrationAndRollbackInvalidatesPlan) {
  Rig h;h.ready();const auto old=h.bus->configure(ConfigurationPhase::status,"","",{},h.now).resolved_configuration;
  auto p=old;p.revision="new";p.ptp_lin_dispatch_mode="synchronized";p.changed.fill(false);
  const auto ready=h.bus->prepare_program(h.program(),h.now);h.enabled_for_settings(false);
  const auto prepared=h.bus->configure(ConfigurationPhase::prepare,"change","bench",p,h.now);ASSERT_TRUE(prepared.accepted)<<prepared.message;
  EXPECT_EQ(prepared.resolved_configuration.calibration_invalid,old.calibration_invalid);
  ASSERT_TRUE(h.bus->configure(ConfigurationPhase::commit,"change","bench",p,h.now).accepted);
  EXPECT_EQ(h.bus->configure(ConfigurationPhase::status,"","",{},h.now).resolved_configuration.ptp_lin_dispatch_mode,"synchronized");
  ASSERT_TRUE(h.bus->configure(ConfigurationPhase::rollback,"change","bench",p,h.now).accepted);
  EXPECT_EQ(h.bus->configure(ConfigurationPhase::status,"","",{},h.now).resolved_configuration.ptp_lin_dispatch_mode,"streamed");
  EXPECT_EQ(h.bus->program_status().state,"invalidated");EXPECT_FALSE(h.bus->execute_program(ready.program_id,"old",h.now));
}
TEST(SynchronizedProgram, IndependentAxisAcknowledgementTimesKeepOneActivation) {
  Rig h;h.ready();h.synchronized();for(unsigned j=1;j<=6;++j) h.port->axis_ack_delay[j]=std::chrono::microseconds(j*500);
  h.execute(h.program());h.run(1600);EXPECT_EQ(h.bus->program_status().state,"succeeded")<<h.bus->program_status().message;
  EXPECT_EQ(h.port->activated_sets,101u);
}
TEST(SynchronizedProgram, CorruptOrDuplicateAcknowledgementLatchesFault) {
  for(bool duplicate:{false,true}) {
    Rig h;h.ready();h.synchronized();h.execute(h.program());wait_for(h,"waiting_ack");
    const auto it=std::find_if(h.port->rx.begin(),h.port->rx.end(),[](const auto &f){return f.data[0]==0xf5;});ASSERT_NE(it,h.port->rx.end());
    if(duplicate) h.port->rx.push_back(*it);else it->data.back()^=1;
    h.run(50);EXPECT_TRUE(h.bus->snapshot().fault);EXPECT_EQ(h.port->counts[0x4b],0u);
  }
}
TEST(SynchronizedProgram, LatenessBoundariesShiftAboveToleranceAndStopAboveLimit) {
  for(unsigned us:{2500,2501,10000,10001}) {
    Rig h;h.ready();h.synchronized();h.execute(h.program());wait_for(h,"waiting_ack");h.run(4);
    h.now=Time{std::chrono::nanoseconds(h.bus->program_status().sync_due_monotonic_ns)}+std::chrono::microseconds(us);
    h.run(3);EXPECT_EQ(h.port->counts[0x4b],us<=10000 ? 1u : 0u) << us;EXPECT_EQ(h.bus->snapshot().fault,us>10000) << us;
    EXPECT_EQ(h.bus->snapshot().statistics.sync_shifts,us>2500 && us<=10000 ? 1u : 0u) << us;
  }
}
TEST(SynchronizedProgram, NextDeadlineIsPreparedSyncTerm) {
  Rig h;h.ready();EXPECT_EQ(h.bus->next_deadline(),Time{});
  h.synchronized();h.execute(h.program());wait_for(h,"waiting_ack");
  EXPECT_EQ(h.bus->next_deadline(),Time{std::chrono::nanoseconds(h.bus->program_status().sync_due_monotonic_ns)});
}
TEST(SynchronizedProgram, FinalRestorationRequiresSpeedMeasuredAfterLastActivation) {
  Rig h;h.ready();h.synchronized();h.execute(h.program());
  for(unsigned i=0;i<1400 && h.port->counts[0x4b]<101;++i) h.run(1);
  ASSERT_EQ(h.port->counts[0x4b],101u);h.port->missing_speed=true;h.run(300);
  EXPECT_NE(h.bus->program_status().state,"succeeded");for(unsigned j=1;j<=6;++j) EXPECT_TRUE(h.port->sync[j]);
  h.port->missing_speed=false;h.run(500);EXPECT_EQ(h.bus->program_status().state,"succeeded")<<h.bus->program_status().message;
}
TEST(SynchronizedProgram, LeftoverSyncAtStartupRunsRecoveryWithoutTriggeringBuffer) {
  Rig h;h.ready();h.synchronized();h.bus->close();h.port->rx.clear();h.bus->reset_session();
  h.port->sync[1]=true;h.port->pending_targets_valid[1]=true;h.port->pending_targets[1]=500;
  h.bus->open(h.now);h.bus->start_scan();h.run(2500);
  EXPECT_TRUE(h.bus->snapshot().fault);EXPECT_TRUE(h.port->sync[1]);
  EXPECT_TRUE(h.port->pending_targets_valid[1]);EXPECT_TRUE(h.port->pending_stops[1]);
  EXPECT_FALSE(h.bus->manual().set_enable(1,true,h.now));EXPECT_EQ(h.port->counts[0x4b],0u);
}
TEST(SynchronizedProgram, LeftoverSyncKeepsDiagnosticsForAllAxesAfterRecoveryGate) {
  Rig h;h.ready();h.synchronized();h.bus->close();h.port->rx.clear();h.bus->reset_session();
  h.port->sync[1]=true;h.bus->open(h.now);h.bus->start_scan();h.run(2500);
  const auto snapshot=h.bus->snapshot();
  EXPECT_TRUE(snapshot.fault);EXPECT_FALSE(h.bus->service_allowed());
  EXPECT_EQ(h.bus->manual().snapshot(h.now).recovery_state,"recovery_failed");
  for(unsigned j=1;j<=6;++j) {
    EXPECT_TRUE(snapshot.devices[j].detected);
    EXPECT_TRUE(snapshot.devices[j].position.fresh(h.now,400ms));
    EXPECT_TRUE(snapshot.devices[j].speed.fresh(h.now,400ms));
    EXPECT_TRUE(snapshot.devices[j].enable.fresh(h.now,400ms));
  }
  EXPECT_EQ(h.port->counts[0x4b],0u);EXPECT_GT(h.port->counts[0xf5],0u);
}
TEST(SynchronizedProgram, FailureBeforeEnablingSyncDoesNotInventUncertainBuffer) {
  Rig h;h.ready();h.synchronized();h.port->missing_speed=true;h.execute(h.program());h.run(300);
  EXPECT_EQ(h.port->counts[0x4b],0u);
  for(unsigned j=1;j<=6;++j) EXPECT_FALSE(h.port->sync[j]);
  h.port->missing_speed=false;h.run(1200);EXPECT_FALSE(h.bus->snapshot().fault)<<h.bus->snapshot().message;
}
TEST(SynchronizedProgram, SharedHorizonUsesOnlyParticipatingAxes) {
  auto c=config();c.program_streaming[0].lookahead_time_sec=.02;
  for(unsigned j=1;j<6;++j) c.program_streaming[j].lookahead_time_sec=.12;
  Rig h(c);h.port->absent=1;h.ready();h.synchronized();auto p=h.program(.2,1,true);p.duration=1;
  const auto expected=program_reference(p,.12).positions[1];h.execute(p);wait_for(h,"waiting_ack");
  EXPECT_EQ(h.port->pending_targets[2],std::llround(expected*10*16384/(2*std::acos(-1.))));
  EXPECT_FALSE(h.port->pending_targets_valid[1]);h.run(1600);
  EXPECT_EQ(h.bus->program_status().state,"succeeded")<<h.bus->program_status().message;
  EXPECT_FALSE(h.bus->program_status().tcp_verified);EXPECT_EQ(h.port->counts[0xf5],505u);
}
TEST(SynchronizedProgram, GlobalStopRecoversWithoutSyncAndKeepsFeedback) {
  auto c=config();c.program_staged_reset_verified=true;Rig h(c);h.ready();h.synchronized();
  for(unsigned j=1;j<=6;++j) {h.port->sync[j]=true;h.port->pending_targets_valid[j]=true;h.port->pending_targets[j]=1000*j;}
  h.bus->stop("STOP operatora",false);h.run(2500);
  const auto state=h.bus->manual().snapshot(h.now);
  EXPECT_FALSE(h.bus->snapshot().fault)<<h.bus->snapshot().message;
  EXPECT_EQ(state.recovery_state,"idle_disabled");EXPECT_EQ(state.recovery_pending_mask,0);
  EXPECT_EQ(h.port->counts[0x4b],0u);
  for(unsigned j=1;j<=6;++j) {
    EXPECT_FALSE(h.port->enabled[j]);EXPECT_FALSE(h.port->sync[j]);EXPECT_FALSE(h.port->respond[j]);
    EXPECT_FALSE(h.port->pending_targets_valid[j]);EXPECT_TRUE(state.axes[j-1].position.valid);
  }
}
TEST(SynchronizedProgram, RecoveryRejectionKeepsMotionBlockedAndDiagnosticsAlive) {
  auto c=config();c.program_staged_reset_verified=true;Rig h(c);h.ready();h.synchronized();
  h.port->sync.fill(true);h.port->rejected_ack=3;h.bus->stop("STOP operatora",false);h.run(2500);
  ASSERT_TRUE(h.bus->snapshot().fault);
  ASSERT_EQ(h.bus->manual().snapshot(h.now).recovery_state,"recovery_failed");
  // Osiom oczekującym przysługuje pojedyncza pauza watchdog+50 ms; potem
  // diagnostyka musi wrócić mimo utrzymanej blokady ruchu.
  h.run(1000);const auto state=h.bus->manual().snapshot(h.now);
  EXPECT_FALSE(h.bus->service_allowed());
  for(unsigned j=0;j<6;++j) {
    EXPECT_TRUE(state.axes[j].position.valid)<<"J"<<j+1;
    EXPECT_TRUE(state.axes[j].speed.valid)<<"J"<<j+1;
    EXPECT_TRUE(state.axes[j].enable.valid)<<"J"<<j+1;
    EXPECT_FALSE(state.axes[j].enable.value)<<"J"<<j+1;
  }
  EXPECT_EQ(h.port->counts[0x4b],0u);
}
TEST(SynchronizedProgram, HardwareAcceptanceGatePreventsAutomaticUnlock) {
  Rig h;h.ready();h.synchronized();h.port->sync.fill(true);h.bus->stop("STOP operatora",false);h.run(2500);
  EXPECT_TRUE(h.bus->snapshot().fault);EXPECT_EQ(h.bus->manual().snapshot(h.now).recovery_state,"recovery_failed");
  EXPECT_NE(h.bus->snapshot().message.find("odbiór sprzętowy"),std::string::npos);
  for(unsigned j=1;j<=6;++j) EXPECT_TRUE(h.port->sync[j]);
  const auto stops=h.port->counts[0xf7];h.run(1000);
  EXPECT_EQ(h.port->counts[0xf7],stops); // diagnostyka nie ponawia odzyskiwania.
  h.bus->stop("Ponowny STOP operatora",false);h.run(2500);
  EXPECT_TRUE(h.bus->snapshot().fault);EXPECT_EQ(h.port->counts[0x4b],0u);
  h.bus->close();h.bus->reset_session(ResetMode::preserve_discovery);h.bus->open(h.now);h.run(1000);
  EXPECT_TRUE(h.bus->snapshot().fault);EXPECT_FALSE(h.bus->manual().set_enable(1,true,h.now));
}
TEST(SynchronizedProgram, CriticalFaultWithAutomaticDisableOffWaitsForOperatorDisable) {
  auto c=config();c.program_staged_reset_verified=true;Rig h(c);h.ready();
  auto profile=h.bus->configure(ConfigurationPhase::status,"","",{},h.now).resolved_configuration;
  profile.revision="operator-disable";profile.disable_enable_on_critical_fault=false;profile.changed.fill(false);
  ASSERT_TRUE(h.bus->configure(ConfigurationPhase::prepare,"operator-disable","bench",profile,h.now).accepted);
  ASSERT_TRUE(h.bus->configure(ConfigurationPhase::commit,"operator-disable","bench",profile,h.now).accepted);
  h.bus->stop("Awaria krytyczna",true);h.run(6000);
  EXPECT_TRUE(h.bus->snapshot().fault);
  EXPECT_EQ(h.bus->recovery_state_name(),"verifying_disabled");
  for(unsigned j=1;j<=6;++j) EXPECT_TRUE(h.port->enabled[j]);
  ASSERT_TRUE(h.bus->manual().set_enable_all(false,h.now));h.run(1800);
  EXPECT_FALSE(h.bus->snapshot().fault)<<h.bus->snapshot().message;
  EXPECT_EQ(h.bus->manual().snapshot(h.now).recovery_state,"idle_disabled");
  for(unsigned j=1;j<=6;++j) EXPECT_FALSE(h.port->enabled[j]);
}
TEST(SynchronizedProgram, TransportReconnectNeverResumesOldSynchronizedTrajectory) {
  auto c=config();c.program_staged_reset_verified=true;Rig h(c);h.ready();h.synchronized();h.execute(h.program());
  for(unsigned i=0;i<300 && h.port->counts[0x4b]<2;++i) h.run(1);
  ASSERT_GE(h.port->counts[0x4b],1u);const auto syncs=h.port->counts[0x4b];
  h.port->disconnected=true;h.run(600);EXPECT_FALSE(h.port->opened);
  h.port->disconnected=false;h.run(3000);
  EXPECT_TRUE(h.port->opened);EXPECT_EQ(h.port->counts[0x4b],syncs);
  EXPECT_FALSE(h.bus->snapshot().fault)<<h.bus->snapshot().message;
  EXPECT_EQ(h.bus->manual().snapshot(h.now).recovery_state,"idle_disabled");
  EXPECT_NE(h.bus->program_status().state,"running");
}
TEST(SynchronizedProgram, HoldingSpeedNoiseOfOneRpmCountsAsStandstill) {
  // Napędy w trzymaniu stale zgłaszają ±1 RPM: uzbrojenie i potwierdzenie końca nie mogą na tym utknąć.
  Rig h;h.ready();h.synchronized();h.port->speed_blips.fill(100000);
  h.execute(h.program());h.run(1800);
  EXPECT_EQ(h.bus->program_status().state,"succeeded") << h.bus->program_status().message << "; " << h.bus->snapshot().message;
}
TEST(SynchronizedProgram, ArmingRereadsTransientMotionAndReportsPersistentOne) {
  for(unsigned blips:{3u,50u}) {
    Rig h;h.ready();h.synchronized();h.port->speed_blip_rpm=2;h.port->speed_blips[6]=blips;
    h.execute(h.program());h.run(1800);
    if(blips<5) {EXPECT_EQ(h.bus->program_status().state,"succeeded") << h.bus->program_status().message;continue;}
    EXPECT_TRUE(h.bus->snapshot().fault);EXPECT_EQ(h.port->counts[0x4b],0u);
    EXPECT_NE(h.bus->snapshot().message.find("J6 (CAN ID 6): niepotwierdzony warunek uzbrojenia/odtwarzania: odczyt prędkości (0x32); odczyt=2 RPM"),std::string::npos)
        << h.bus->snapshot().message;
  }
}
TEST(SynchronizedProgram, FinalApproachCorrectsDriveOffsetWithSyncSetsAndTimesOutWhenStuck) {
  {
    // Przesunięcie układu F5 J3 ≈ 1,1° przegubu (> 0,5°): dolot SYNC koryguje cel i kończy programem udanym.
    Rig h;h.ready();h.synchronized();h.port->drive_offset[3]=500;
    h.execute(h.program());h.run(3000);
    const auto s=h.bus->program_status();EXPECT_EQ(s.state,"succeeded") << s.message << "; " << h.bus->snapshot().message;
    EXPECT_GT(h.port->counts[0x4b],101u) << "zestawy korekcyjne wyzwalane SYNC";
    EXPECT_LE(std::abs(static_cast<double>(h.port->positions[3]-std::llround(.2*10*16384/(2*std::acos(-1.))))),1.);
  }
  {
    // Oś zablokowana po trajektorii: zamiast wiecznego oczekiwania awaria po limicie dolotu.
    Rig h;h.ready();h.synchronized();h.port->drive_offset[2]=1000;h.port->frozen[2]=false;h.execute(h.program());
    for(unsigned i=0;i<3000 && h.bus->program_status().state!="waiting_position" && !h.bus->snapshot().fault;++i) h.run(1);
    ASSERT_EQ(h.bus->program_status().state,"waiting_position") << h.bus->program_status().message;
    h.port->frozen[2]=true;h.run(7000);EXPECT_TRUE(h.bus->snapshot().fault);
    EXPECT_NE(h.bus->program_status().message.find("SYNC: dolot końcowy nieudany"),std::string::npos) << h.bus->program_status().message;
  }
}
namespace {
void set_position_integral(Rig &h,bool enabled) {
  h.enabled_for_settings(false);
  auto p=h.bus->configure(ConfigurationPhase::status,"","",{},h.now).resolved_configuration;
  const auto previous=p.revision;p.revision=enabled ? "integral-on" : "integral-off";p.position_integral=enabled;p.changed.fill(false);
  if(!h.bus->configure(ConfigurationPhase::prepare,"integral",previous,p,h.now).accepted ||
      !h.bus->configure(ConfigurationPhase::commit,"integral",previous,p,h.now).accepted ||
      !h.bus->configure(ConfigurationPhase::finish,"integral",previous,p,h.now).accepted) throw std::runtime_error("integral-settings");
  h.enabled_for_settings(true);
}
}
TEST(MinDither, ConfigurationValueReachesF5SpeedAndRejectsInvalidValues) {
  Rig h;h.ready();h.enabled_for_settings(false);
  auto p=h.bus->configure(ConfigurationPhase::status,"","",{},h.now).resolved_configuration;
  EXPECT_DOUBLE_EQ(p.min_dither_rpm,kDefaultMinDitherRpm);
  const auto previous=p.revision;p.changed.fill(false);
  for(double bad:{0.,-1.,10.5,static_cast<double>(NAN)}) {
    auto q=p;q.revision="dither-bad";q.min_dither_rpm=bad;
    EXPECT_FALSE(h.bus->configure(ConfigurationPhase::prepare,"dither-bad",previous,q,h.now).accepted) << bad;
  }
  p.revision="dither-2";p.min_dither_rpm=2;
  ASSERT_TRUE(h.bus->configure(ConfigurationPhase::prepare,"dither",previous,p,h.now).accepted);
  ASSERT_TRUE(h.bus->configure(ConfigurationPhase::commit,"dither",previous,p,h.now).accepted);
  ASSERT_TRUE(h.bus->configure(ConfigurationPhase::finish,"dither",previous,p,h.now).accepted);
  EXPECT_DOUBLE_EQ(h.bus->configure(ConfigurationPhase::status,"","",{},h.now).resolved_configuration.min_dither_rpm,2);
  h.enabled_for_settings(true);
  h.execute(h.program(.2,2.));h.run(300);
  unsigned frames=0;
  for(const auto &e:h.port->events) if(e.data.size()==8 && e.data[0]==0xf5) {
    ++frames;EXPECT_GE(((e.data[1]&0x0f)<<8)|e.data[2],2/speed_scale(h.port->microsteps[e.axis])-1e-9) << "minimum 2 RPM wału";
  }
  EXPECT_GT(frames,0u);
}
TEST(F5Acceleration, ConfigurationCodeOverridesExecutionCodeInF5Frames) {
  for(std::uint8_t code:{0,200}) {
    Rig h;h.ready();h.enabled_for_settings(false);
    auto p=h.bus->configure(ConfigurationPhase::status,"","",{},h.now).resolved_configuration;
    EXPECT_EQ(p.f5_acceleration_code,0);
    const auto previous=p.revision;p.changed.fill(false);p.revision="acc";p.f5_acceleration_code=code;
    ASSERT_TRUE(h.bus->configure(ConfigurationPhase::prepare,"acc",previous,p,h.now).accepted);
    ASSERT_TRUE(h.bus->configure(ConfigurationPhase::commit,"acc",previous,p,h.now).accepted);
    ASSERT_TRUE(h.bus->configure(ConfigurationPhase::finish,"acc",previous,p,h.now).accepted);
    EXPECT_EQ(h.bus->configure(ConfigurationPhase::status,"","",{},h.now).resolved_configuration.f5_acceleration_code,code);
    h.enabled_for_settings(true);
    h.execute(h.program(.2,2.));h.run(300);
    unsigned frames=0;
    for(const auto &e:h.port->events) if(e.data.size()==8 && e.data[0]==0xf5) {
      ++frames;
      if(code) EXPECT_EQ(e.data[3],code) << "acc z konfiguracji robota";
      else EXPECT_NE(e.data[3],0) << "acc z parametrów wykonania";
    }
    EXPECT_GT(frames,0u);
  }
}
TEST(PositionIntegral, ConfigurationOptionReachesF5TargetsAndCompensatesDriveOffset) {
  std::array<std::int64_t,2> late{};
  for(bool enabled:{false,true}) {
    Rig h;h.ready();set_position_integral(h,enabled);
    EXPECT_EQ(h.bus->configure(ConfigurationPhase::status,"","",{},h.now).resolved_configuration.position_integral,enabled);
    h.port->drive_offset[2]=200;  // napęd stale 200 impulsów przed celem F5 (≈0,44° przegubu)
    h.execute(h.program(.2,2.));h.run(1500);
    // Cel F5 J2 po ~1,5 s ruchu: z członem całkującym przesunięty w dół o narosłą korektę.
    for(auto it=h.port->events.rbegin();it!=h.port->events.rend();++it) if(it->axis==2 && it->data[0]==0xf5) {
      const auto raw=(std::uint32_t(it->data[4])<<16)|(std::uint32_t(it->data[5])<<8)|it->data[6];
      late[enabled]=raw&0x800000 ? static_cast<std::int32_t>(raw)-0x1000000 : static_cast<std::int32_t>(raw);break;
    }
    h.run(3000);EXPECT_EQ(h.bus->program_status().state,"succeeded") << h.bus->program_status().message;
  }
  EXPECT_LT(late[1],late[0]-100) << "korekta ~200 impulsów w stronę zmniejszenia celu";
}
TEST(HoldGuard, DriftWithoutCommandAfterIdleTriggersCriticalStop) {
  Rig h;h.ready();h.run(300);ASSERT_FALSE(h.bus->snapshot().fault);
  h.port->positions[4]+=30;h.run(200);
  EXPECT_FALSE(h.bus->snapshot().fault) << "szum/dobieg poniżej progu nie jest awarią: " << h.bus->snapshot().message;
  h.port->positions[4]+=200;h.run(200);
  ASSERT_TRUE(h.bus->snapshot().fault);
  EXPECT_NE(h.bus->snapshot().message.find("ruch bez komendy w postoju"),std::string::npos) << h.bus->snapshot().message;
  EXPECT_NE(h.bus->snapshot().message.find("J4"),std::string::npos) << h.bus->snapshot().message;
  bool f7=false;for(const auto &e:h.port->events) f7=f7 || (e.axis==4 && e.data[0]==0xf7);
  EXPECT_TRUE(f7) << "awaria krytyczna wysyła F7";
}
TEST(HoldGuard, ProgramMotionIsNotSupervisedAndAnchorFollowsFinalPosition) {
  Rig h;h.ready();h.execute(h.program(.2,2.));h.run(4000);
  ASSERT_EQ(h.bus->program_status().state,"succeeded") << h.bus->program_status().message;
  h.run(500);EXPECT_FALSE(h.bus->snapshot().fault) << h.bus->snapshot().message;
  h.port->positions[2]-=300;h.run(200);
  EXPECT_TRUE(h.bus->snapshot().fault) << "odjazd po zakończonym programie (stara komenda F5 napędu)";
}
TEST(HoldGuard, DisabledAxisMayMoveAndIsReanchoredAfterEnable) {
  Rig h;h.ready();h.run(300);
  h.port->enabled[3]=false;h.run(400);
  h.port->positions[3]+=5000;h.run(200);
  EXPECT_FALSE(h.bus->snapshot().fault) << "ENABLE=0: wał wolny, bez nadzoru; " << h.bus->snapshot().message;
  h.port->enabled[3]=true;h.run(400);
  EXPECT_FALSE(h.bus->snapshot().fault) << "po ENABLE=1 nowa kotwica z bieżącej pozycji; " << h.bus->snapshot().message;
  h.port->positions[3]+=500;h.run(200);
  EXPECT_TRUE(h.bus->snapshot().fault);
}

TEST(IsolatedProgram, SingleAxisMaskCommandsOnlyThatAxisWithOthersDisabled) {
  for(bool synchronized:{false,true}) {
    Rig h;h.ready();if(synchronized) h.synchronized();
    for(unsigned j=1;j<=6;++j) if(j!=4) h.port->enabled[j]=false;
    h.run(400);
    auto p=h.program();const auto state=h.bus->control_state();
    for(unsigned j=0;j<6;++j) p.points.back().positions[j]=j==3 ? state.positions[3].value+.05 : state.positions[j].value;
    p.participation_mask=1u<<3;
    const auto events=h.port->events.size();
    const auto status=h.bus->prepare_program(p,h.now);
    ASSERT_EQ(status.state,"ready") << status.message;
    EXPECT_FALSE(status.tcp_verified);
    for(unsigned j=0;j<6;++j) EXPECT_EQ(status.participating[j],j==3);
    ASSERT_TRUE(h.bus->execute_program(status.program_id,"iso",h.now));h.run(3000);
    EXPECT_EQ(h.bus->program_status().state,"succeeded") << h.bus->program_status().message;
    EXPECT_FALSE(h.bus->snapshot().fault) << h.bus->snapshot().message;
    for(auto i=events;i<h.port->events.size();++i) {
      const auto &e=h.port->events[i];
      if(e.data[0]==0xf5) EXPECT_EQ(e.axis,4u) << "F5 tylko do osi z maski";
    }
  }
  Rig g;g.port->absent=4;g.ready();auto q=g.program();q.participation_mask=1u<<3;
  EXPECT_THROW(g.bus->prepare_program(q,g.now),std::runtime_error) << "oś z maski niewykryta";
}
TEST(SettleTimeout, StuckMotorStateFailsProgramWithAxisDetail) {
  Rig h;h.ready();h.synchronized();
  h.execute(h.program(.2,1.));
  for(unsigned i=0;i<4000 && h.bus->program_status().state!="settling";++i) h.run(1);
  ASSERT_EQ(h.bus->program_status().state,"settling");
  h.port->motor_state_override[2]=3;  // J2: F1=3 (hamowanie) bez końca, mimo postoju
  h.run(5000);
  ASSERT_TRUE(h.bus->snapshot().fault) << h.bus->program_status().state << " " << h.bus->program_status().message;
  EXPECT_NE(h.bus->snapshot().message.find("Brak potwierdzenia postoju"),std::string::npos) << h.bus->snapshot().message;
  EXPECT_NE(h.bus->snapshot().message.find("J2: RPM=0, F1=3"),std::string::npos) << h.bus->snapshot().message;
}
TEST(SynchronizedProgram, LongProgramKeepsBudgetForEverySyncBatch) {
  // Diagnostyka w tle nie może zająć miejsca zestawu SYNC (F5+ACK+zapytanie pozycji na oś + ramka SYNC).
  Rig h;h.ready();h.synchronized();h.execute(h.program(.2,25.));
  h.run(28000);
  EXPECT_FALSE(h.bus->snapshot().fault) << h.bus->snapshot().message;
  EXPECT_EQ(h.bus->program_status().state,"succeeded") << h.bus->program_status().message;
  EXPECT_LE(h.bus->snapshot().statistics.peak_scheduled_utilization,kPlannedUtilization+1e-9);
}
TEST(ExecutionTuning, RuntimeChangeAppliesToF5RejectsDuringMotionAndInvalidatesProgram) {
  ExecutionTuning bad{.3,1,2.5,2,.01};std::string reason;
  {Rig h;h.ready();EXPECT_FALSE(h.bus->set_execution_tuning(bad,reason));EXPECT_FALSE(reason.empty());}
  std::array<double,2> speed{};
  for(int variant=0;variant<2;++variant) {
    Rig h;h.ready();
    if(variant) ASSERT_TRUE(h.bus->set_execution_tuning({.085,2.,2.5,2.,.0174},reason)) << reason;
    h.execute(h.program(.2,2.));h.run(600);
    EXPECT_FALSE(h.bus->set_execution_tuning({.085,1.,2.5,2.,.0174},reason)) << "zmiana w trakcie ruchu";
    unsigned n=0;double sum=0;
    for(const auto &e:h.port->events) if(e.axis==2 && e.data.size()==8 && e.data[0]==0xf5) {sum+=((e.data[1]&0x0f)<<8)|e.data[2];++n;}
    ASSERT_GT(n,0u);speed[variant]=sum/n;
  }
  EXPECT_GT(speed[1],speed[0]*1.5) << "FF=2 podnosi prędkość F5";
  Rig h;h.ready();
  const auto status=h.bus->prepare_program(h.program(.2,2.),h.now);
  ASSERT_EQ(h.bus->program_status().state,"ready") << h.bus->program_status().message;
  ASSERT_TRUE(h.bus->set_execution_tuning({.12,1.,2.5,2.,.0174},reason)) << reason;
  EXPECT_EQ(h.bus->program_status().state,"invalidated");
  EXPECT_FALSE(h.bus->execute_program(status.program_id,"run",h.now));
}
TEST(ExecutionTuning, TestAccelerationOverrideIncludingZeroReachesF5) {
  for(int code:{0,-1}) {
    Rig h;h.ready();std::string reason;
    ExecutionTuning t{.085,1.,2.5,2.,.0174};t.acceleration_code=code;
    ASSERT_TRUE(h.bus->set_execution_tuning(t,reason)) << reason;
    h.execute(h.program(.2,2.));h.run(300);
    unsigned n=0;
    for(const auto &e:h.port->events) if(e.data.size()==8 && e.data[0]==0xf5) {++n;if(code==0) EXPECT_EQ(e.data[3],0); else EXPECT_NE(e.data[3],0);}
    EXPECT_GT(n,0u);
  }
  Rig h;h.ready();std::string reason;ExecutionTuning bad{.085,1.,2.5,2.,.0174};bad.acceleration_code=256;
  EXPECT_FALSE(h.bus->set_execution_tuning(bad,reason));
}
TEST(RunawayGuard, ShiftedDriveFrameTriggersFaultWithAngleErrorReading) {
  Rig h;h.ready();h.synchronized();h.execute(h.program(.2,4.));h.run(1500);
  ASSERT_FALSE(h.bus->snapshot().fault) << h.bus->snapshot().message;
  // Układ celu F5 napędu J2 przesunięty o −20000 imp. (≈ −44° przegubu): napęd „ucieka” od referencji.
  h.port->drive_offset[2]=-20000;h.port->angle_error[2]=62500;  // 39h: 439,45° wału
  h.run(300);
  ASSERT_TRUE(h.bus->snapshot().fault);
  const auto message=h.bus->program_status().message;
  EXPECT_NE(message.find("ucieczka osi"),std::string::npos) << message;
  EXPECT_NE(message.find("J2"),std::string::npos) << message;
  EXPECT_NE(message.find("39h"),std::string::npos) << message;
  EXPECT_NE(message.find("439"),std::string::npos) << message;
}
TEST(RunawayGuard, NormalSynchronizedProgramsDoNotTrigger) {
  for(double target:{.2,-.3,.6}) {
    Rig h;h.ready();h.synchronized();h.execute(h.program(target,2.));h.run(4000);
    EXPECT_FALSE(h.bus->snapshot().fault) << h.bus->snapshot().message;
    EXPECT_EQ(h.bus->program_status().state,"succeeded") << h.bus->program_status().message;
  }
}
