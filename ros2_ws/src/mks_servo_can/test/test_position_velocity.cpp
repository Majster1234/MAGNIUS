#include <gtest/gtest.h>
#include <cmath>
#include "mks_servo_can/position_velocity.hpp"
using namespace mks_servo_can;
using namespace std::chrono_literals;
namespace {
using Time=PositionHistory::Time;
constexpr double kTicksPerRpmSecond=16384./60;  // impulsy wału na sekundę przy 1 RPM
// Próbki co period od start do end (bez end), pozycja z funkcji czasu [s].
template<class F> Time fill(PositionHistory &h,Time start,std::chrono::milliseconds period,unsigned count,F position) {
  Time t=start;
  for(unsigned i=0;i<count;++i,t+=period) h.push(std::llround(position(std::chrono::duration<double>(t-start).count())),t);
  return t-period;
}
}
TEST(PositionVelocity, StandingShaftIsStillAndCreepIsNot) {
  PositionHistory still;const auto last=fill(still,Time{10s},10ms,60,[](double) {return 1000000.;});
  const auto a=still.estimate(last+5ms);
  ASSERT_TRUE(a.valid) << a.reason;EXPECT_NEAR(a.rpm,0,1e-9);EXPECT_EQ(a.samples,51u);EXPECT_TRUE(a.still(1));
  // Pełzanie 1,5 RPM wału mieści się w paśmie tolerancji dolotu, ale nie jest postojem.
  PositionHistory creep;const auto end=fill(creep,Time{10s},10ms,60,[](double t) {return -1.5*kTicksPerRpmSecond*t;});
  const auto b=creep.estimate(end);
  ASSERT_TRUE(b.valid) << b.reason;EXPECT_NEAR(b.rpm,-1.5,.02);EXPECT_FALSE(b.still(1));
}
TEST(PositionVelocity, HoldingOscillationAveragesOutButDriftDoesNot) {
  // J4 w trzymaniu drga ±55 imp. wału (11.11.2026): nachylenie z 500 ms zostaje poniżej 1 RPM; dryf 2 RPM nie.
  constexpr double kPi=3.14159265358979323846;
  PositionHistory h;const auto last=fill(h,Time{1s},10ms,80,[&](double t) {return 55*std::sin(2*kPi*5*t);});
  const auto a=h.estimate(last);ASSERT_TRUE(a.valid) << a.reason;EXPECT_TRUE(a.still(1)) << a.rpm;
  PositionHistory d;const auto end=fill(d,Time{1s},10ms,80,[&](double t) {return 55*std::sin(2*kPi*5*t)+2*kTicksPerRpmSecond*t;});
  const auto b=d.estimate(end);ASSERT_TRUE(b.valid) << b.reason;EXPECT_FALSE(b.still(1)) << b.rpm;
}
TEST(PositionVelocity, IdlePollingOf40msIsEnough) {
  PositionHistory h;const auto last=fill(h,Time{1s},40ms,20,[](double) {return 5.;});
  const auto a=h.estimate(last+30ms);ASSERT_TRUE(a.valid) << a.reason;EXPECT_EQ(a.samples,13u);
}
TEST(PositionVelocity, MissingDataNeverReadsAsStandstill) {
  PositionHistory empty;EXPECT_FALSE(empty.estimate(Time{1s}).valid);
  // Nieświeża ostatnia próbka.
  PositionHistory stale;const auto last=fill(stale,Time{1s},10ms,60,[](double) {return 0.;});
  EXPECT_FALSE(stale.estimate(last+101ms).valid);EXPECT_TRUE(stale.estimate(last+100ms).valid);
  EXPECT_FALSE(stale.estimate(last-1ms).valid) << "chwila oceny przed ostatnią próbką";
  // Przerwa 150 ms w oknie.
  PositionHistory gap;auto t=fill(gap,Time{1s},10ms,30,[](double) {return 0.;});
  t=fill(gap,t+150ms,10ms,30,[](double) {return 0.;});
  const auto g=gap.estimate(t);EXPECT_FALSE(g.valid);EXPECT_NE(g.reason.find("przerwa"),std::string::npos) << g.reason;
  // Niepełne okno (historia 300 ms) i za mało próbek.
  PositionHistory short_window;const auto s=fill(short_window,Time{1s},10ms,31,[](double) {return 0.;});
  EXPECT_FALSE(short_window.estimate(s).valid);
  PositionHistory sparse;const auto p=fill(sparse,Time{1s},90ms,7,[](double) {return 0.;});
  EXPECT_FALSE(sparse.estimate(p).valid);
}
TEST(PositionVelocity, NonIncreasingStampsAreIgnoredAndBufferWraps) {
  PositionHistory h;h.push(0,Time{1s});h.push(500,Time{1s});h.push(900,Time{1s}-1ms);EXPECT_EQ(h.size(),1u);
  h.clear();EXPECT_EQ(h.size(),0u);
  // 300 próbek > pojemność: zostają najnowsze, nachylenie poprawne mimo dużych liczników.
  const auto last=fill(h,Time{1s},10ms,300,[](double t) {return 4e12+2*kTicksPerRpmSecond*t;});
  EXPECT_EQ(h.size(),PositionHistory::kCapacity);
  const auto a=h.estimate(last);ASSERT_TRUE(a.valid) << a.reason;EXPECT_NEAR(a.rpm,2,.01);
}
TEST(PositionVelocity, StepAndMotionOnsetAfterFlatHistoryAreNotStandstill) {
  // Skok pozycji w oknie nie jest postojem: 150 imp. — nachylenie > 1 RPM; 300 imp. — odchyłka od prostej > 100 imp.
  PositionHistory step;auto t=fill(step,Time{1s},10ms,40,[](double) {return 0.;});
  t=fill(step,t+10ms,10ms,20,[](double) {return -150.;});
  const auto a=step.estimate(t);EXPECT_FALSE(a.still(1)) << "rpm=" << a.rpm << " " << a.reason;
  PositionHistory big;t=fill(big,Time{1s},10ms,40,[](double) {return 0.;});
  t=fill(big,t+10ms,10ms,20,[](double) {return -300.;});
  const auto c=big.estimate(t);EXPECT_FALSE(c.valid);EXPECT_NE(c.reason.find("nieliniowy"),std::string::npos) << c.reason;
  // Początek ruchu 4 RPM po postoju: po 150 ms nachylenie z 500 ms jest rozmyte (< 1 RPM), z ostatnich 200 ms > 3 RPM.
  PositionHistory onset;t=fill(onset,Time{1s},10ms,40,[](double) {return 0.;});
  const auto start=t;
  for(unsigned i=1;i<=15;++i) onset.push(std::llround(4*kTicksPerRpmSecond*.01*i),start+std::chrono::milliseconds(10*i));
  const auto b=onset.estimate(start+150ms);
  EXPECT_FALSE(b.still(1)) << "rpm=" << b.rpm << " recent=" << b.recent_rpm << " " << b.reason;
}
