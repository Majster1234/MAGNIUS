#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <time.h>
#include <vector>

namespace robot_benchmarks {
using Clock = std::chrono::steady_clock;
struct Sample { std::uint64_t wall_ns, cpu_ns; };

inline std::uint64_t cpu_time_ns() {
  timespec value{};
  if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &value) != 0)
    throw std::runtime_error("Brak zegara CPU procesu");
  return std::uint64_t(value.tv_sec) * 1000000000ULL + value.tv_nsec;
}

template<class Prepare>
Sample measure(Prepare prepare, std::uint64_t iterations) {
  auto state = prepare();
  const auto cpu_start = cpu_time_ns();
  const auto wall_start = Clock::now();
  for (std::uint64_t index = 0; index < iterations; ++index) state.cycle(index);
  const auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now() - wall_start).count();
  const auto cpu_ns = cpu_time_ns() - cpu_start;
  if (!state.verify()) throw std::runtime_error("Nieprawidłowy wynik scenariusza");
  return {static_cast<std::uint64_t>(wall_ns), cpu_ns};
}

struct Result { std::uint64_t iterations; std::vector<Sample> samples; };

template<class Prepare>
Result execute(Prepare prepare, std::uint64_t iterations = 0,
               unsigned samples = 7, double warmup_seconds = 1.0,
               std::uint64_t min_series_ns = 200000000ULL) {
  if (!samples || warmup_seconds < 0 || !min_series_ns)
    throw std::invalid_argument("Nieprawidłowe parametry pomiaru");
  auto count = iterations ? iterations : 1;
  const auto warmup_end = Clock::now() + std::chrono::duration<double>(warmup_seconds);
  while (true) {
    const auto trial = measure(prepare, count);
    if (!iterations && trial.wall_ns < min_series_ns) {
      const auto factor = std::min<std::uint64_t>(10, std::max<std::uint64_t>(2,
          (min_series_ns + std::max<std::uint64_t>(1, trial.wall_ns) - 1) /
          std::max<std::uint64_t>(1, trial.wall_ns)));
      if (count > 1000000000ULL / factor)
        throw std::runtime_error("Przekroczono limit liczby cykli");
      count *= factor;
      continue;
    }
    if (Clock::now() >= warmup_end) break;
  }
  Result result{count, {}};
  for (unsigned i = 0; i < samples; ++i) result.samples.push_back(measure(prepare, count));
  return result;
}

// Wywołanie w osobnym procesie. Runner narzuca limit czasu całemu procesowi.
template<class Prepare>
int main(int argc, char **argv, Prepare prepare) {
  try {
    std::uint64_t iterations = 0;
    if (argc != 1) {
      if (argc != 3 || std::string(argv[1]) != "--iterations")
        throw std::invalid_argument("Oczekiwano --iterations N");
      const std::string value(argv[2]);
      if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
        throw std::invalid_argument("Nieprawidłowa liczba cykli");
      iterations = std::stoull(value);
    }
    const auto result = execute(prepare, iterations);
    std::cout << "{\"iterations\":" << result.iterations << ",\"samples\":[";
    bool first = true;
    for (const auto &sample : result.samples) {
      if (!first) std::cout << ',';
      first = false;
      std::cout << "{\"wall_ns\":" << sample.wall_ns
                << ",\"cpu_ns\":" << sample.cpu_ns << '}';
    }
    std::cout << "]}\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
}  // namespace robot_benchmarks
