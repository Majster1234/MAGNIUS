#include "mks_servo_can/realtime.hpp"
#include <cstring>
#include <sched.h>
#include <stdexcept>
#include <sys/prctl.h>

namespace mks_servo_can {
std::string set_realtime_priority(std::thread &thread, int priority) {
  if(priority<=0) return {};
  const int low=sched_get_priority_min(SCHED_FIFO),high=sched_get_priority_max(SCHED_FIFO);
  if(priority<low || priority>high) return "Priorytet SCHED_FIFO poza zakresem "+std::to_string(low)+"–"+std::to_string(high);
  sched_param parameters{};parameters.sched_priority=priority;
  const int error=pthread_setschedparam(thread.native_handle(),SCHED_FIFO,&parameters);
  if(error) return "SCHED_FIFO "+std::to_string(priority)+" odrzucony: "+std::strerror(error)+" (wymagane LimitRTPRIO w usłudze)";
  return {};
}
void reduce_timer_slack() noexcept { prctl(PR_SET_TIMERSLACK,1000UL,0,0,0); }
PriorityInheritanceMutex::PriorityInheritanceMutex() {
  pthread_mutexattr_t attributes;
  if(pthread_mutexattr_init(&attributes)) throw std::runtime_error("pthread_mutexattr_init");
  const int error=pthread_mutexattr_setprotocol(&attributes,PTHREAD_PRIO_INHERIT) || pthread_mutex_init(&mutex_,&attributes);
  pthread_mutexattr_destroy(&attributes);
  if(error) throw std::runtime_error("Mutex z dziedziczeniem priorytetu niedostępny");
}
PriorityInheritanceMutex::~PriorityInheritanceMutex() { pthread_mutex_destroy(&mutex_); }
void PriorityInheritanceMutex::lock() { if(pthread_mutex_lock(&mutex_)) throw std::runtime_error("pthread_mutex_lock"); }
bool PriorityInheritanceMutex::try_lock() { return pthread_mutex_trylock(&mutex_)==0; }
void PriorityInheritanceMutex::unlock() { pthread_mutex_unlock(&mutex_); }
}  // namespace mks_servo_can
