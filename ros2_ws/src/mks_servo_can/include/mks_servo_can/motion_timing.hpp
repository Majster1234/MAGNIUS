#pragma once
#include <chrono>
#include <cstddef>

namespace mks_servo_can {
// Jeden profil czasu dla planera, kompilatora oraz właściciela CAN.
inline constexpr std::chrono::milliseconds kMotionCycle{10};
inline constexpr double kMotionCycleSeconds = .010;
inline constexpr std::chrono::milliseconds kStreamingCycle{10};
inline constexpr double kStreamingCycleSeconds=.010;
inline constexpr std::chrono::milliseconds kStageWindow{8};
inline constexpr std::chrono::milliseconds kFeedbackWindowEnd{14};
inline constexpr std::chrono::milliseconds kDiagnosticWindowEnd{18};
inline constexpr std::chrono::milliseconds kEncoderPeriod{40};
// Bez żadnej nadanej ramki dłużej niż limit: zamknięcie gniazda i ponowne otwarcie.
inline constexpr std::chrono::milliseconds kTransmitQueueFullLimit{500};
// JOG TCP: F5 tylko na początku slotu 10 ms od startu sesji, przed odczytami tego slotu.
inline constexpr std::chrono::milliseconds kTcpSendWindow{2};
// Odczyt w slocie TCP kończy się 1 ms przed kolejnym F5; brak odpowiedzi nie blokuje następnego slotu.
inline constexpr std::chrono::milliseconds kTcpReplyMargin{1};
inline constexpr std::chrono::milliseconds kProgramAckTimeout{10};
inline constexpr std::chrono::milliseconds kProgramPositionPeriod{10};
inline constexpr std::chrono::milliseconds kProgramFeedbackTimeout{250};
// Dolot końcowy streamed: pierwsza korekta po ustaleniu ostatniej ramki, kolejne co odczyt, limit całkowity.
inline constexpr std::chrono::milliseconds kFinalApproachDelay{300};
inline constexpr std::chrono::milliseconds kFinalApproachPeriod{200};
inline constexpr std::chrono::milliseconds kFinalApproachTimeout{5000};
// MKS 0x32 w trzymaniu pozycji zgłasza stale ±1 RPM (drgania o kilka impulsów, próba 03.11.2026);
// postój SYNC = |RPM| ≤ 1 razem z F1=1.
inline constexpr int kHoldingSpeedNoiseRpm = 1;
// Maks. czas oczekiwania na potwierdzenie postoju (|RPM| ≤ 1, F1=1) po dolocie; potem awaria z opisem osi
// (07.11.2026: napęd w stanie F1=3 bez końca zawiesił program na ≥ 29 s).
inline constexpr std::chrono::milliseconds kSettleTimeout{3000};
inline constexpr double kProgramPositionTolerance = 0.008726646259971648;  // 0,5° przegubu
// Nadzór postoju: maks. wiek pomiaru pozycji użytego do porównania i dolny próg odchyłki (1° wału),
// powyżej szumu trzymania MKS (kilka impulsów) i dobiegu po potwierdzeniu postoju.
inline constexpr std::chrono::milliseconds kHoldFeedbackAge{100};
inline constexpr double kHoldMinimumTicks = 45.;
inline constexpr unsigned kScanMaximumAddress = 20;
inline constexpr std::size_t kDiagnosticTransactionsPerCycle = 8;
inline constexpr double kPlannedUtilization = .4;
// Nadzór ucieczki osi (SYNC, 08.11.2026; progi z nagrań testów jakości TCP: normalnie ≤ 1,2° przy acc ≥ 241).
inline constexpr double kRunawayTrackingLimitRad = 0.06981317007977318;   // 4° przegubu — twardy limit uchybu śledzenia
inline constexpr double kRunawayReverseErrorRad = 0.02617993877991494;    // 1,5° — uchyb, od którego sprawdzany jest kierunek
inline constexpr double kRunawayReverseMoveRad = 0.005235987755982988;    // 0,3° oddalenia w 200 ms bez hamowania
inline constexpr double kRunawayReferenceMoveRad = 0.0008726646259971648; // 0,05° — poniżej referencja uznawana za stojącą
inline constexpr double kRunawayFinalGrowthRad = 0.003490658503988659;    // 0,2° wzrostu uchybu między korektami dolotu
inline constexpr double kRunawayFinalMinRad = 0.017453292519943295;       // 1° — minimalny uchyb dla kryterium dolotu
inline constexpr std::chrono::milliseconds kRunawayProbeTimeout{40};      // odczyt 39h przed awarią
// Robot zachowuje 40%; dodatkowe 20 punktów procentowych należy do przyszłego modułu.
inline constexpr double kFutureModuleReservedUtilization = .2;
inline constexpr double kTotalPlannedUtilization = .6;
static_assert(kPlannedUtilization + kFutureModuleReservedUtilization <= kTotalPlannedUtilization + 1e-12);
inline constexpr double can_frame_bits(std::size_t dlc) { return 55. + 10. * dlc; }
}
