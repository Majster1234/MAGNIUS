#include "mks_servo_can/socketcan.hpp"

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

namespace mks_servo_can {
SocketCan::~SocketCan() { close(); }
void SocketCan::open(const std::string &interface) {
  close();
  fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (fd_ < 0) throw std::runtime_error(std::strerror(errno));
  int own_messages = 0;
  const can_err_mask_t errors = CAN_ERR_BUSOFF | CAN_ERR_CRTL | CAN_ERR_PROT | CAN_ERR_ACK |
                                CAN_ERR_BUSERROR | CAN_ERR_RESTARTED;
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &own_messages, sizeof(own_messages)) ||
      ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors))) {
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
}
void SocketCan::close() { if (fd_ >= 0) ::close(fd_); fd_ = -1; }
bool SocketCan::is_open() const { return fd_ >= 0; }
void SocketCan::send(std::uint16_t address, const std::vector<std::uint8_t> &data) {
  if (!is_open()) throw std::runtime_error("socket CAN nie jest otwarty");
  if (address > CAN_SFF_MASK || data.size() > CAN_MAX_DLEN) throw std::invalid_argument("nieprawidłowa ramka CAN");
  can_frame frame{};
  frame.can_id = address;
  frame.can_dlc = static_cast<__u8>(data.size());
  std::copy(data.begin(), data.end(), frame.data);
  if (::write(fd_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
    throw std::runtime_error(std::strerror(errno));
  }
}
std::optional<Frame> SocketCan::receive() {
  if (!is_open()) return std::nullopt;
  can_frame raw{};
  iovec io{&raw, sizeof(raw)};
  msghdr message{};
  message.msg_iov = &io; message.msg_iovlen = 1;
  const auto received = ::recvmsg(fd_, &message, MSG_DONTWAIT);
  if (received < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return std::nullopt;
    throw std::runtime_error(std::strerror(errno));
  }
  if (received != static_cast<ssize_t>(sizeof(raw))) return std::nullopt;
  // Kernel does not deliver our own sends (CAN_RAW_RECV_OWN_MSGS=0). MSG_DONTROUTE
  // is a second guard for unusual local loopback configurations.
  if (message.msg_flags & MSG_DONTROUTE) return std::nullopt;
  Frame result;
  result.error = (raw.can_id & CAN_ERR_FLAG) != 0;
  result.error_flags = raw.can_id & CAN_ERR_MASK;
  result.address = static_cast<std::uint16_t>(raw.can_id & CAN_SFF_MASK);
  result.data.assign(raw.data, raw.data + raw.can_dlc);
  return result;
}
}  // namespace mks_servo_can
