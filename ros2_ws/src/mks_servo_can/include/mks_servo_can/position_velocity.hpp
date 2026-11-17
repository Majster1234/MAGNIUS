#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

namespace mks_servo_can {
// Prędkość wału z kolejnych odczytów pozycji 0x31 (17.11.2026). MKS 1.0.9 po zatrzymaniu potrafi przez 1–2 s zwracać
// w 0x32 ostatnią prędkość ruchu (nagranie JOG TCP 17.11: −12 → −2 RPM przy wale stojącym w ±1 imp.). Postój
// potwierdzamy wtedy nachyleniem prostej MNK dopasowanej do pozycji z okna — drgania trzymania (J4 ±55 imp.) uśredniają
// się, rzeczywisty dryf nie. Błąd groźny to brak danych (prędkość 0 z powodu braku próbek), stąd warunki jakości.
// Prosta MNK z długiego okna nie widzi skoku ani początku ruchu po płaskiej historii (nachylenie „rozmyte” przez
// stojącą część okna). Dlatego także: maks. odchyłka próbki od prostej (przebieg nieliniowy = brak oceny) oraz nachylenie
// z ostatniej części okna z luźniejszym progiem (recent_factor × próg), powyżej szumu drgań trzymania w krótkim oknie.
struct PositionVelocityLimits {
  std::chrono::milliseconds window{500};     // okno dopasowania kończące się na najnowszej próbce
  std::chrono::milliseconds max_gap{100};    // maks. przerwa między kolejnymi próbkami (bezczynność: odczyt co 40 ms)
  std::chrono::milliseconds max_age{100};    // maks. wiek najnowszej próbki względem chwili oceny
  unsigned min_samples{8};
  double max_residual_ticks{100};            // J4 w trzymaniu drga ±55 imp. wału (11.11.2026); 0 = bez sprawdzenia
  std::chrono::milliseconds recent_window{200};  // 0 = bez sprawdzenia części końcowej
  double recent_factor{3};                   // próg części końcowej = recent_factor × próg postoju
};
// Ocena postoju: okno 500 ms (≥ 8 próbek, bez przerw > 100 ms, pokrycie ≥ okno − przerwa).
inline constexpr PositionVelocityLimits kStandstillVelocityLimits{};
// Porównanie z 0x32 (diagnostyka): krótkie okno, małe opóźnienie; w bezczynności 2–3 próbki; bez kontroli kształtu.
inline constexpr PositionVelocityLimits kComparisonVelocityLimits{std::chrono::milliseconds(100),
    std::chrono::milliseconds(60),std::chrono::milliseconds(100),3,0,std::chrono::milliseconds(0),0};

struct PositionVelocity {
  bool valid{};            // warunki jakości spełnione
  double rpm{};            // nachylenie z całego okna [RPM wału], ze znakiem; tylko przy valid
  double recent_rpm{};     // nachylenie z części końcowej okna (recent_window)
  double residual_ticks{}; // maks. |próbka − prosta| w oknie
  double recent_factor{};  // z limitów oceny
  bool recent_valid{};
  unsigned samples{};      // liczba próbek w oknie
  std::string reason;      // przyczyna braku oceny
  bool still(double limit_rpm) const {
    return valid && std::abs(rpm) <= limit_rpm && (!recent_valid || std::abs(recent_rpm) <= recent_factor * limit_rpm);
  }
};

class PositionHistory {
 public:
  using Time = std::chrono::steady_clock::time_point;
  static constexpr std::size_t kCapacity = 128;  // 100 Hz w ruchu: 50 próbek na okno 500 ms, z zapasem

  void clear() { size_ = 0; head_ = 0; }
  std::size_t size() const { return size_; }
  // Próbki starsze od najnowszej (lub równe) są pomijane: znacznik czasu musi rosnąć.
  void push(std::int64_t ticks, Time stamp) {
    if (size_ && stamp <= at(size_ - 1).stamp) return;
    samples_[(head_ + size_) % kCapacity] = {ticks, stamp};
    if (size_ < kCapacity) ++size_; else head_ = (head_ + 1) % kCapacity;
  }
  PositionVelocity estimate(Time now, const PositionVelocityLimits &limits = kStandstillVelocityLimits) const {
    PositionVelocity out;
    if (!size_) { out.reason = "brak próbek pozycji"; return out; }
    const auto &newest = at(size_ - 1);
    if (now < newest.stamp || now - newest.stamp > limits.max_age) { out.reason = "nieświeża pozycja"; return out; }
    const auto begin = newest.stamp - limits.window;
    std::size_t first = size_ - 1;
    while (first > 0 && at(first - 1).stamp >= begin) --first;
    out.samples = static_cast<unsigned>(size_ - first);
    if (out.samples < limits.min_samples) { out.reason = "za mało próbek pozycji ("+std::to_string(out.samples)+")"; return out; }
    if (newest.stamp - at(first).stamp < limits.window - limits.max_gap) { out.reason = "niepełne okno pozycji"; return out; }
    for (std::size_t i = first + 1; i < size_; ++i)
      if (at(i).stamp - at(i - 1).stamp > limits.max_gap) { out.reason = "przerwa w odczytach pozycji"; return out; }
    const auto line = fit(first);
    if (!line.valid) { out.reason = "zerowy rozrzut czasu próbek"; return out; }
    out.rpm = line.rpm;
    for (std::size_t i = first; i < size_; ++i)
      out.residual_ticks = std::max(out.residual_ticks, std::abs(static_cast<double>(at(i).ticks - newest.ticks) - line.at(seconds(at(i).stamp - newest.stamp))));
    if (limits.max_residual_ticks > 0 && out.residual_ticks > limits.max_residual_ticks) {
      out.reason = "nieliniowy przebieg pozycji (odchyłka "+std::to_string(std::lround(out.residual_ticks))+" imp.)"; return out;
    }
    if (limits.recent_window.count() > 0) {
      std::size_t recent = size_ - 1;
      while (recent > first && at(recent - 1).stamp >= newest.stamp - limits.recent_window) --recent;
      if (size_ - recent >= 3) {
        const auto tail = fit(recent);
        out.recent_valid = tail.valid; out.recent_rpm = tail.rpm; out.recent_factor = limits.recent_factor;
      }
    }
    out.valid = true;
    return out;
  }

 private:
  struct Line {
    bool valid{}; double rpm{}, slope{}, mean_t{}, mean_x{};
    double at(double t) const { return mean_x + slope * (t - mean_t); }
  };
  // MNK od próbki first do najnowszej, we współrzędnych względem najnowszej (bez utraty precyzji przy dużych licznikach).
  Line fit(std::size_t first) const {
    Line line; const auto &newest = at(size_ - 1);
    double st = 0, sx = 0; const double n = static_cast<double>(size_ - first);
    for (std::size_t i = first; i < size_; ++i) { st += seconds(at(i).stamp - newest.stamp); sx += static_cast<double>(at(i).ticks - newest.ticks); }
    line.mean_t = st / n; line.mean_x = sx / n;
    double stt = 0, stx = 0;
    for (std::size_t i = first; i < size_; ++i) {
      const double t = seconds(at(i).stamp - newest.stamp) - line.mean_t, x = static_cast<double>(at(i).ticks - newest.ticks) - line.mean_x;
      stt += t * t; stx += t * x;
    }
    if (!(stt > 0)) return line;
    line.slope = stx / stt; line.rpm = line.slope * 60.0 / 16384.0;  // 16384 imp./obrót wału
    line.valid = std::isfinite(line.rpm);
    return line;
  }
  struct Sample { std::int64_t ticks{}; Time stamp{}; };
  static double seconds(Time::duration d) { return std::chrono::duration<double>(d).count(); }
  const Sample &at(std::size_t i) const { return samples_[(head_ + i) % kCapacity]; }
  std::array<Sample, kCapacity> samples_{};
  std::size_t head_{}, size_{};
};
}  // namespace mks_servo_can
