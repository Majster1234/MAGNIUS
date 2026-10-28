#pragma once

#include "mks_servo_can/bus_manager.hpp"
#include <iomanip>
#include <locale>
#include <sstream>

namespace mks_servo_can {
inline std::string diagnostic_number(double value) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(3) << value;
  return out.str();
}

inline std::string axis_diagnostic_label(unsigned joint, std::uint16_t address) {
  return "J" + std::to_string(joint) + " (CAN ID " + std::to_string(address) + ")";
}

// Opis próbki z chwili wykrycia błędu, przed unieważnieniem danych po STOP.
template<class T>
std::string sample_diagnostic(const Sample<T> &sample, Time now,
    std::chrono::milliseconds limit, const std::string &name) {
  if (!sample.valid) return name + ": brak ważnego odczytu";
  const auto age = std::chrono::duration<double, std::milli>(now - sample.stamp).count();
  if (now < sample.stamp)
    return name + ": znacznik czasu z przyszłości (wiek " + diagnostic_number(age) + " ms)";
  return name + ": nieaktualny odczyt, wiek " + diagnostic_number(age) +
      " ms, wymagany < " + std::to_string(limit.count()) + " ms";
}
}
