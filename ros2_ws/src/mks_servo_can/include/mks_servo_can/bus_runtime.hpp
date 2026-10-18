#pragma once
#include <atomic>
#include <mutex>
#include <thread>
#include "mks_servo_can/bus_manager.hpp"
#include "mks_servo_can/manual_motor_controller.hpp"

namespace mks_servo_can {
// Właściciel wątku IO. Interfejsy try_* nie czekają na mutex ani transport.
class BusRuntime {
 public:
  explicit BusRuntime(BusConfig config, std::unique_ptr<Transport> transport = std::make_unique<SocketCan>());
  ~BusRuntime();
  void start(bool scan = true);
  void shutdown();
  void reset_session();
  bool scan();
  bool reset_allowed();
  bool activate(Source source);
  void stop(const std::string &reason);
  void renew_source_lease(Source source, Time created);
  bool try_submit(const CommandBatch &batch);
  bool try_read(ControlState &state);
  BusSnapshot snapshot();
  ManualState manual_snapshot();
  bool manual_enable(unsigned axis, bool enabled);
  bool manual_zero(unsigned axis);
  bool manual_jog(JogAction action, unsigned axis, int direction, unsigned rpm,
                  const std::string &session, std::uint64_t sequence, Time issued);
 private:
  std::mutex mutex_;
  CanBusManager manager_;
  std::atomic<bool> running_{false};
  std::thread worker_;
};
}  // namespace mks_servo_can
