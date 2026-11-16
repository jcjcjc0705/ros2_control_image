// can_tool: candump/cansend that works through any hw_transport plugin, including boards
// that aren't SocketCAN (serial USB-CAN adapters). Useful for checking a board folder's
// transport before any motor code exists.
//
//   ros2 run hw_transport can_tool dump <type> <port> [key=value ...]
//   ros2 run hw_transport can_tool send <type> <port> <frame>... [-n count] [-r hz] [-w ms] [-q]
//                                       [key=value ...]
//
//   <type>    transport plugin, e.g. hw_transport/SocketCan
//   <frame>   cansend syntax, e.g. 123#0102AA; several frames are sent in order
//   -n count  repeat the frames this many times (1)
//   -r hz     repeat rate (100)
//   -w ms     keep printing received frames this long after the last send (200)
//   -g us     gap between the frames of one repetition (0)
//   -q        don't print frames, only the totals
//
// `send` prints what comes back in the same process, so it also works with transports that
// open their port exclusively (serial adapters), where a separate `dump` can't run alongside.
// `send <frame> -n 6000 -r 6000 -q` is a quick bus-load test.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hw_transport/registry.hpp"

namespace
{
std::atomic<bool> g_stop{false};

void on_signal(int) {g_stop = true;}

int usage()
{
  std::fprintf(
    stderr,
    "usage:\n"
    "  can_tool dump <type> <port> [key=value ...]\n"
    "  can_tool send <type> <port> <frame>... [-n count] [-r hz] [-w ms] [-g us] [-q] [key=value ...]\n"
    "example:\n"
    "  can_tool dump hw_transport/SocketCan can0\n"
    "  can_tool send hw_transport/SocketCan can0 001#FFFFFFFFFFFFFFFC\n");
  return 2;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 4) {return usage();}
  const std::string command = argv[1];
  const std::string type = argv[2];
  const std::string port = argv[3];
  if (command != "dump" && command != "send") {return usage();}

  std::vector<hw_transport::CanFrame> frames;
  long count = 1;
  double rate = 100.0;
  int wait_ms = 200;
  int gap_us = 0;
  bool quiet = false;
  hw_transport::Params params;
  const std::vector<std::string> rest(argv + 4, argv + argc);
  for (std::size_t i = 0; i < rest.size(); ++i) {
    hw_transport::CanFrame frame;
    if (rest[i] == "-n" && i + 1 < rest.size()) {
      count = std::atol(rest[++i].c_str());
    } else if (rest[i] == "-r" && i + 1 < rest.size()) {
      rate = std::atof(rest[++i].c_str());
    } else if (rest[i] == "-w" && i + 1 < rest.size()) {
      wait_ms = std::atoi(rest[++i].c_str());
    } else if (rest[i] == "-g" && i + 1 < rest.size()) {
      gap_us = std::atoi(rest[++i].c_str());
    } else if (rest[i] == "-q") {
      quiet = true;
    } else if (command == "send" && hw_transport::parse_can_frame(rest[i], frame)) {
      frames.push_back(frame);
    } else if (rest[i].find('=') != std::string::npos && rest[i].find('#') == std::string::npos) {
      const auto eq = rest[i].find('=');
      params[rest[i].substr(0, eq)] = rest[i].substr(eq + 1);
    } else {
      std::fprintf(stderr, "can't parse '%s'\n", rest[i].c_str());
      return usage();
    }
  }
  if (command == "send" && frames.empty()) {
    std::fprintf(stderr, "no frame given, expected e.g. 123#0102AA\n");
    return 2;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  std::shared_ptr<hw_transport::CanBus> bus;
  try {
    bus = hw_transport::acquire_can_bus(type, port, params);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }

  const auto start = std::chrono::steady_clock::now();
  std::mutex print_mutex;
  std::atomic<long> received{0};
  bus->subscribe_all(
    [&](const hw_transport::CanFrame & f) {
      ++received;
      if (quiet) {return;}
      const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      std::lock_guard<std::mutex> lock(print_mutex);
      std::printf("(%10.6f)  %s  %s\n", t, port.c_str(), hw_transport::to_string(f).c_str());
      std::fflush(stdout);
    });

  if (command == "dump") {
    while (!g_stop) {std::this_thread::sleep_for(std::chrono::milliseconds(100));}
    return 0;
  }

  const auto period = std::chrono::duration<double>(rate > 0 ? 1.0 / rate : 0.0);
  auto next = std::chrono::steady_clock::now();
  long sent = 0, failed = 0;
  for (long n = 0; n < count && !g_stop; ++n) {
    for (std::size_t f = 0; f < frames.size(); ++f) {
      if (f > 0 && gap_us > 0) {std::this_thread::sleep_for(std::chrono::microseconds(gap_us));}
      (bus->send(frames[f]) ? sent : failed)++;
    }
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next);
  }
  const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
  while (!g_stop && std::chrono::steady_clock::now() < until) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::lock_guard<std::mutex> lock(print_mutex);
  std::printf("sent %ld, refused %ld, received %ld\n", sent, failed, received.load());
  return failed == 0 ? 0 : 1;
}
