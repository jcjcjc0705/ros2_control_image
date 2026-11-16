#include "hw_transport/serial_port.hpp"

#include <fcntl.h>
#include <linux/serial.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace hw_transport
{

namespace
{
speed_t to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1152000: return B1152000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 2500000: return B2500000;
    case 3000000: return B3000000;
    case 3500000: return B3500000;
    case 4000000: return B4000000;
    default:
      throw std::invalid_argument("unsupported baud rate " + std::to_string(baud));
  }
}

std::runtime_error errno_error(const std::string & what)
{
  return std::runtime_error(what + ": " + std::strerror(errno));
}
}  // namespace

SerialPort::~SerialPort() {close();}

void SerialPort::open(const std::string & path, int baud)
{
  if (fd_ >= 0) {throw std::runtime_error(path_ + " is already open");}
  const speed_t speed = to_speed(baud);

  const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {throw errno_error("can't open " + path);}

  // Exclusive, like pyserial's exclusive=True (same flock): two programs on one port would
  // each get half of the received frames, and both could command the same device.
  if (::flock(fd, LOCK_EX | LOCK_NB) < 0) {
    ::close(fd);
    throw std::runtime_error(
            path + " is in use by another program (check with: fuser -v " + path + ")");
  }

  termios tio{};
  if (::tcgetattr(fd, &tio) < 0) {
    const auto error = errno_error("tcgetattr " + path);
    ::close(fd);
    throw error;
  }
  ::cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~(CSTOPB | CRTSCTS | PARENB | CSIZE);
  tio.c_cflag |= CS8;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  ::cfsetispeed(&tio, speed);
  ::cfsetospeed(&tio, speed);
  if (::tcsetattr(fd, TCSANOW, &tio) < 0) {
    const auto error = errno_error("tcsetattr " + path);
    ::close(fd);
    throw error;
  }

  // Best effort: FTDI-style adapters otherwise hold received bytes for up to 16 ms.
  serial_struct serial{};
  if (::ioctl(fd, TIOCGSERIAL, &serial) == 0) {
    serial.flags |= ASYNC_LOW_LATENCY;
    ::ioctl(fd, TIOCSSERIAL, &serial);
  }

  ::tcflush(fd, TCIOFLUSH);
  fd_ = fd;
  path_ = path;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::write(const std::uint8_t * data, std::size_t size, int timeout_ms)
{
  if (fd_ < 0) {return false;}
  std::size_t done = 0;
  while (done < size) {
    const ssize_t n = ::write(fd_, data + done, size - done);
    if (n > 0) {
      done += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno != EAGAIN && errno != EINTR) {return false;}
    pollfd pfd{fd_, POLLOUT, 0};
    if (::poll(&pfd, 1, timeout_ms) <= 0) {return false;}
  }
  return true;
}

ssize_t SerialPort::read(std::uint8_t * buffer, std::size_t size, int timeout_ms)
{
  if (fd_ < 0) {return -1;}
  pollfd pfd{fd_, POLLIN, 0};
  const int ready = ::poll(&pfd, 1, timeout_ms);
  if (ready < 0) {return errno == EINTR ? 0 : -1;}
  if (ready == 0) {return 0;}
  if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {return -1;}
  const ssize_t n = ::read(fd_, buffer, size);
  if (n < 0) {return (errno == EAGAIN || errno == EINTR) ? 0 : -1;}
  return n;
}

void SerialPort::flush()
{
  if (fd_ >= 0) {::tcflush(fd_, TCIOFLUSH);}
}

}  // namespace hw_transport
