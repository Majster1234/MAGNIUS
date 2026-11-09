#include "harness.hpp"
#include <thread>

using namespace std::chrono_literals;

struct CheckedCycle {
  unsigned count{0};
  void cycle(std::uint64_t) { ++count; }
  bool verify() const {
    std::this_thread::sleep_for(30ms);
    return count == 5;
  }
};
struct InvalidCycle {
  void cycle(std::uint64_t) {}
  bool verify() const { return false; }
};

int main() {
  const auto start = robot_benchmarks::Clock::now();
  const auto sample = robot_benchmarks::measure([] {
    std::this_thread::sleep_for(30ms);
    return CheckedCycle{};
  }, 5);
  const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
      robot_benchmarks::Clock::now() - start).count();
  // Koszt przygotowania i weryfikacji musi pozostać poza odcinkiem pomiarowym.
  if (elapsed - sample.wall_ns < 60000000ULL) return 1;
  try {
    robot_benchmarks::measure([] { return InvalidCycle{}; }, 1);
    return 2;
  } catch (const std::runtime_error &) {}
  const auto result = robot_benchmarks::execute([] { return CheckedCycle{}; }, 5, 2, 0);
  if (result.iterations != 5 || result.samples.size() != 2) return 3;
  return 0;
}
