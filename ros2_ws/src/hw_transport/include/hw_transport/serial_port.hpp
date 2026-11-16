#pragma once

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hw_transport
{

/**
 * Raw 8N1 serial port for transports whose board shows up as a tty (USB-CAN adapters that
 * use their own framing over a virtual COM port, RS-485 converters, MCU boards, ...).
 * Non-blocking underneath; every call takes a timeout so it never stalls a control loop
 * for longer than asked.
 */
class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  /// Opens `path` in raw mode at `baud` (standard Linux rates, 9600 to 4000000).
  /// Throws std::runtime_error / std::invalid_argument on failure.
  void open(const std::string & path, int baud);
  void close();
  bool is_open() const {return fd_ >= 0;}
  const std::string & path() const {return path_;}

  /// Writes all of `data`, waiting at most `timeout_ms` whenever the driver's buffer is full.
  /// Returns false on error or timeout.
  bool write(const std::uint8_t * data, std::size_t size, int timeout_ms = 100);
  bool write(const std::vector<std::uint8_t> & data, int timeout_ms = 100)
  {
    return write(data.data(), data.size(), timeout_ms);
  }

  /// Reads up to `size` bytes, waiting at most `timeout_ms` for the first one.
  /// Returns the number of bytes read, 0 on timeout, or -1 if the device failed or was
  /// unplugged.
  ssize_t read(std::uint8_t * buffer, std::size_t size, int timeout_ms);

  /// Discards anything not yet sent or read.
  void flush();

private:
  int fd_ = -1;
  std::string path_;
};

}  // namespace hw_transport
