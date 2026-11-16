// can_tool: candump/cansend that works through any hw_transport plugin, including boards
// that aren't SocketCAN (serial USB-CAN adapters). Useful for checking a board folder's
// transport before any motor code exists.
//
//   ros2 run hw_transport can_tool dump <type> <port> [key=value ...]
//   ros2 run hw_transport can_tool send <type> <port> <frame> [-n count] [-r hz] [key=value ...]
//
//   <type>   transport plugin, e.g. hw_transport/SocketCan
//   <frame>  cansend syntax, e.g. 123#0102AA
//
// `send -n 6000 -r 6000` is a quick bus-load test: it reports how many frames the
// transport refused.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
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
    "  can_tool send <type> <port> <frame> [-n count] [-r hz] [key=value ...]\n"
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

  hw_transport::CanFrame frame;
  long count = 1;
  double rate = 100.0;
  hw_transport::Params params;
  std::vector<std::string> rest(argv + 4, argv + argc);

  std::size_t i = 0;
  if (command == "send") {
    if (rest.empty() || !hw_transport::parse_can_frame(rest[0], frame)) {
      std::fprintf(stderr, "invalid frame, expected e.g. 123#0102AA\n");
      return 2;
    }
    i = 1;
  } else if (command != "dump") {
    return usage();
  }
  for (; i < rest.size(); ++i) {
    if (rest[i] == "-n" && i + 1 < rest.size()) {
      count = std::atol(rest[++i].c_str());
    } else if (rest[i] == "-r" && i + 1 < rest.size()) {
      rate = std::atof(rest[++i].c_str());
    } else if (rest[i].find('=') != std::string::npos) {
      const auto eq = rest[i].find('=');
      params[rest[i].substr(0, eq)] = rest[i].substr(eq + 1);
    } else {
      return usage();
    }
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

  if (command == "dump") {
    const auto start = std::chrono::steady_clock::now();
    bus->subscribe_all(
      [&](const hw_transport::CanFrame & f) {
        const double t =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        std::printf("(%10.6f)  %s  %s\n", t, port.c_str(), hw_transport::to_string(f).c_str());
        std::fflush(stdout);
      });
    while (!g_stop) {std::this_thread::sleep_for(std::chrono::milliseconds(100));}
    return 0;
  }

  const auto period = std::chrono::duration<double>(rate > 0 ? 1.0 / rate : 0.0);
  auto next = std::chrono::steady_clock::now();
  long sent = 0, failed = 0;
  for (long n = 0; n < count && !g_stop; ++n) {
    (bus->send(frame) ? sent : failed)++;
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next);
  }
  std::printf("sent %ld, refused %ld\n", sent, failed);
  return failed == 0 ? 0 : 1;
}
