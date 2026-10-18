#include "mks_servo_can/socketcan.hpp"
#include "mks_servo_can/trace.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/file.h>

namespace mks_servo_can {
SocketCan::~SocketCan() { close(); }
void SocketCan::open(const std::string &interface) {
  close();
  if (interface.empty() || interface.size() >= IFNAMSIZ ||
      interface.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != std::string::npos)
    throw std::invalid_argument("nieprawidłowa nazwa interfejsu CAN");
  // Wspólna blokada dla procesu diagnostyki i pluginu hardware tego konta.
  const auto lock_path = "/tmp/robot-can-" + std::to_string(::getuid()) + "-" + interface + ".lock";
  lock_fd_ = ::open(lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
  if (lock_fd_ < 0 || ::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
    close(); throw std::runtime_error("interfejs CAN ma już właściciela lub blokada jest niedostępna");
  }
  fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (fd_ < 0) { const std::string error = std::strerror(errno); close(); throw std::runtime_error(error); }
  int own_messages = 0;
  int timestamps = 1;
  const can_err_mask_t errors = CAN_ERR_BUSOFF | CAN_ERR_CRTL | CAN_ERR_PROT | CAN_ERR_ACK |
                                CAN_ERR_BUSERROR | CAN_ERR_RESTARTED;
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &own_messages, sizeof(own_messages)) ||
      ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) ||
      ::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS, &timestamps, sizeof(timestamps)) ||
      ::setsockopt(fd_, SOL_SOCKET, SO_RXQ_OVFL, &timestamps, sizeof(timestamps))) {
    const auto error = std::strerror(errno); close(); throw std::runtime_error(error);
  }
  ifreq request{};
  std::strncpy(request.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &request) < 0) {
    const auto error = std::strerror(errno); close(); throw std::runtime_error(error);
  }
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    const auto error = std::strerror(errno); close(); throw std::runtime_error(error);
  }
  trace_note("socket_open", interface);
}
void SocketCan::close() {
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
  if (lock_fd_ >= 0) ::close(lock_fd_);
  lock_fd_ = -1;
  rx_overflow_ = 0;
}
bool SocketCan::is_open() const { return fd_ >= 0; }
void SocketCan::send(std::uint16_t address, const std::vector<std::uint8_t> &data) {
  if (!is_open()) throw std::runtime_error("socket CAN nie jest otwarty");
  if (address > CAN_SFF_MASK || data.size() > CAN_MAX_DLEN) throw std::invalid_argument("nieprawidłowa ramka CAN");
  can_frame frame{};
  frame.can_id = address;
  frame.can_dlc = static_cast<__u8>(data.size());
  std::copy(data.begin(), data.end(), frame.data);
  const auto started = std::chrono::steady_clock::now();
  if (::write(fd_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
    trace_note("can_send_error", std::strerror(errno), address, data.empty() ? 0 : data[0]);
    throw std::runtime_error(std::strerror(errno));
  }
  TraceEvent e; e.event = "can_tx"; e.axis = address; e.command = data.empty() ? 0 : data[0];
  e.duration_ns = trace_ns(std::chrono::steady_clock::now()) - trace_ns(started);
  e.dlc = data.size(); std::copy(data.begin(), data.end(), e.data.begin()); trace(e);
}
std::optional<Frame> SocketCan::receive() {
  if (!is_open()) return std::nullopt;
  can_frame raw{};
  iovec io{&raw, sizeof(raw)};
  msghdr message{};
  alignas(cmsghdr) std::array<char, CMSG_SPACE(sizeof(timespec)) + CMSG_SPACE(sizeof(std::uint32_t))> control{};
  message.msg_control = control.data(); message.msg_controllen = control.size();
  message.msg_iov = &io; message.msg_iovlen = 1;
  const auto received = ::recvmsg(fd_, &message, MSG_DONTWAIT);
  if (received < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return std::nullopt;
    throw std::runtime_error(std::strerror(errno));
  }
  if (received != static_cast<ssize_t>(sizeof(raw))) return std::nullopt;
  // Własne echo jest wyłączone. Inne lokalne sockety mogą być emulatorem vcan;
  // dopasowanie adresu/opcode/DLC/CRC w managerze odrzuca echo zapytań.
  if ((message.msg_flags & MSG_CONFIRM) ||
      (raw.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)) || raw.can_dlc > CAN_MAX_DLEN)
    return std::nullopt;
  Frame result;
  result.error = (raw.can_id & CAN_ERR_FLAG) != 0;
  result.error_flags = raw.can_id & CAN_ERR_MASK;
  result.address = static_cast<std::uint16_t>(raw.can_id & CAN_SFF_MASK);
  result.data.assign(raw.data, raw.data + raw.can_dlc);
  result.received_at = std::chrono::steady_clock::now();
  for (auto *c = CMSG_FIRSTHDR(&message); c; c = CMSG_NXTHDR(&message, c)) {
    if (c->cmsg_level != SOL_SOCKET) continue;
    if (c->cmsg_type == SO_TIMESTAMPNS && c->cmsg_len >= CMSG_LEN(sizeof(timespec))) {
      timespec stamp; std::memcpy(&stamp, CMSG_DATA(c), sizeof(stamp));
      result.kernel_utc_ns = static_cast<std::int64_t>(stamp.tv_sec) * 1000000000 + stamp.tv_nsec;
    }
    if (c->cmsg_type == SO_RXQ_OVFL && c->cmsg_len >= CMSG_LEN(sizeof(std::uint32_t))) {
      std::uint32_t dropped; std::memcpy(&dropped, CMSG_DATA(c), sizeof(dropped));
      if (dropped != rx_overflow_) {
        TraceEvent e; e.event = "socket_rx_overflow"; e.flags = dropped; trace(e); rx_overflow_ = dropped;
      }
    }
  }
  TraceEvent e; e.event = result.error ? "can_error_frame" : "can_rx";
  e.mono_ns = trace_ns(result.received_at); e.kernel_utc_ns = result.kernel_utc_ns;
  e.axis = result.address; e.command = result.data.empty() ? 0 : result.data[0]; e.flags = result.error_flags;
  e.dlc = result.data.size(); std::copy(result.data.begin(), result.data.end(), e.data.begin()); trace(e);
  return result;
}
}  // namespace mks_servo_can
