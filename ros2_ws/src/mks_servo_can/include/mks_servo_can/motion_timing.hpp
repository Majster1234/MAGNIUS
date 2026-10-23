#pragma once
#include <chrono>
#include <cstddef>

namespace mks_servo_can {
// Jeden profil czasu dla planera, kompilatora oraz właściciela CAN.
inline constexpr std::chrono::milliseconds kMotionCycle{20};
inline constexpr double kMotionCycleSeconds = .020;
inline constexpr std::chrono::milliseconds kStageWindow{8};
inline constexpr std::chrono::milliseconds kFeedbackWindowEnd{14};
inline constexpr std::chrono::milliseconds kDiagnosticWindowEnd{18};
inline constexpr std::chrono::milliseconds kEncoderPeriod{40};
inline constexpr unsigned kScanMaximumAddress = 20;
inline constexpr std::size_t kDiagnosticTransactionsPerCycle = 8;
inline constexpr double kPlannedUtilization = .4;
inline constexpr double can_frame_bits(std::size_t dlc) { return 55. + 10. * dlc; }
}
