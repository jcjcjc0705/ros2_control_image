// hw_transport/SocketCan: Linux SocketCAN (can0, vcan0, ...). Covers every adapter that
// has a kernel driver: candleLight/gs_usb, PEAK, Kvaser, Jetson and Raspberry Pi CAN
// controllers. Bring the interface up on the host first, e.g.
//   sudo ip link set can0 up type can bitrate 1000000

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

#include "hw_transport/can_bus.hpp"

namespace hw_transport
{

class SocketCan : public CanBus
{
public:
  ~SocketCan() override {close();}

  void open(const std::string & port, const Params & /*params*/) override
  {
    if (fd_ >= 0) {throw std::runtime_error("SocketCan: " + port_ + " is already open");}
    if (port.empty() || port.size() >= IFNAMSIZ) {
      throw std::runtime_error("SocketCan: invalid interface name '" + port + "'");
    }

    const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {throw error("socket");}

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, port.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
      const auto e = error("no CAN interface '" + port + "'");
      ::close(fd);
      throw e;
    }

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      const auto e = error("bind " + port);
      ::close(fd);
      throw e;
    }

    // Non-blocking: a full TX queue makes send() return false instead of stalling write().
    ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK);

    fd_ = fd;
    port_ = port;
    running_ = true;
    rx_thread_ = std::thread([this] {rx_loop();});
  }

  void close() override
  {
    running_ = false;
    if (rx_thread_.joinable()) {rx_thread_.join();}
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool is_open() const override {return fd_ >= 0;}

  bool send(const CanFrame & frame) override
  {
    if (fd_ < 0 || frame.len > 8) {return false;}
    can_frame cf{};
    cf.can_id = frame.id & (frame.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    if (frame.extended) {cf.can_id |= CAN_EFF_FLAG;}
    if (frame.remote) {cf.can_id |= CAN_RTR_FLAG;}
    cf.len = frame.len;
    std::memcpy(cf.data, frame.data.data(), frame.len);
    return ::write(fd_, &cf, sizeof(cf)) == static_cast<ssize_t>(sizeof(cf));
  }

private:
  static std::runtime_error error(const std::string & what)
  {
    return std::runtime_error("SocketCan: " + what + ": " + std::strerror(errno));
  }

  void rx_loop()
  {
    bool reported = false;  // log a failing interface once, not every 100 ms
    pollfd pfd{fd_, POLLIN, 0};
    while (running_) {
      const int ready = ::poll(&pfd, 1, 100);
      if (ready == 0 || (ready < 0 && errno == EINTR)) {continue;}

      can_frame cf{};
      const ssize_t n = (ready > 0 && (pfd.revents & POLLIN)) ? ::read(fd_, &cf, sizeof(cf)) : -1;
      if (n != static_cast<ssize_t>(sizeof(cf))) {
        if (n < 0 && errno == EAGAIN) {continue;}
        if (!reported) {
          RCLCPP_ERROR(
            rclcpp::get_logger("hw_transport"), "SocketCan %s: receive failed: %s",
            port_.c_str(), std::strerror(errno));
          reported = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      reported = false;
      if (cf.can_id & CAN_ERR_FLAG) {continue;}

      CanFrame frame;
      frame.extended = cf.can_id & CAN_EFF_FLAG;
      frame.remote = cf.can_id & CAN_RTR_FLAG;
      frame.id = cf.can_id & (frame.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
      frame.len = std::min<std::uint8_t>(cf.len, 8);
      std::memcpy(frame.data.data(), cf.data, frame.len);
      dispatch(frame);
    }
  }

  int fd_ = -1;
  std::string port_;
  std::atomic<bool> running_{false};
  std::thread rx_thread_;
};

}  // namespace hw_transport

PLUGINLIB_EXPORT_CLASS(hw_transport::SocketCan, hw_transport::CanBus)
