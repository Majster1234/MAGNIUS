#include "harness.hpp"

struct Cycle {
  // volatile zapewnia obserwowalną pracę nawet przy kompilacji Release.
  volatile std::uint32_t value{0};
  std::uint64_t count{0};
  void cycle(std::uint64_t index) {
    value = value + static_cast<std::uint32_t>(index + 1);
    ++count;
  }
  bool verify() const {
    return value == static_cast<std::uint32_t>(count * (count + 1) / 2);
  }
};

int main(int argc, char **argv) {
  return robot_benchmarks::main(argc, argv, [] { return Cycle{}; });
}
