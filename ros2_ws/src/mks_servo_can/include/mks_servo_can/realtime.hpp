#pragma once
#include <pthread.h>
#include <string>
#include <thread>

namespace mks_servo_can {
// Wątki IO i RX CAN: SCHED_FIFO wymaga LimitRTPRIO (systemd) albo CAP_SYS_NICE. Bez uprawnień wątek
// zostaje w SCHED_OTHER, a wynik opisuje komunikat. priority=0 wyłącza zmianę.
std::string set_realtime_priority(std::thread &thread, int priority);
// Timer slack 1 µs bieżącego wątku: sleep_until budzi bliżej terminu SYNC (domyślnie do 50 µs później).
void reduce_timer_slack() noexcept;
// Mutex z dziedziczeniem priorytetu: wątek ROS trzymający blokadę przejmuje priorytet czekającego wątku IO,
// więc nie wywłaszczy go inny zwykły wątek (inwersja priorytetów przed terminem SYNC).
class PriorityInheritanceMutex {
 public:
  PriorityInheritanceMutex();
  ~PriorityInheritanceMutex();
  PriorityInheritanceMutex(const PriorityInheritanceMutex &) = delete;
  PriorityInheritanceMutex &operator=(const PriorityInheritanceMutex &) = delete;
  void lock();
  bool try_lock();
  void unlock();
 private:
  pthread_mutex_t mutex_;
};
}  // namespace mks_servo_can
