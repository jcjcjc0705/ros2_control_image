#include "hw_transport/registry.hpp"

#include <map>
#include <mutex>
#include <stdexcept>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/logging.hpp>

namespace hw_transport
{

namespace
{
struct Entry
{
  std::string type;
  Params params;
  std::weak_ptr<CanBus> bus;
};

// Function-local statics: one instance per process, shared by every hardware component,
// because they all link against this library. Only weak_ptrs live here, so buses are
// destroyed by their holders (hardware components), never at static destruction.
std::mutex & registry_mutex()
{
  static std::mutex mutex;
  return mutex;
}

std::map<std::string, Entry> & registry()
{
  static std::map<std::string, Entry> entries;
  return entries;
}

pluginlib::ClassLoader<CanBus> & loader()
{
  static pluginlib::ClassLoader<CanBus> class_loader("hw_transport", "hw_transport::CanBus");
  return class_loader;
}

rclcpp::Logger logger() {return rclcpp::get_logger("hw_transport");}
}  // namespace

std::shared_ptr<CanBus> acquire_can_bus(
  const std::string & type, const std::string & port, const Params & params)
{
  std::lock_guard<std::mutex> lock(registry_mutex());
  Entry & entry = registry()[port];

  if (auto bus = entry.bus.lock()) {
    if (entry.type != type) {
      throw std::runtime_error(
              "port '" + port + "' is already open as '" + entry.type + "', not '" + type + "'");
    }
    if (entry.params != params) {
      RCLCPP_WARN(
        logger(), "port '%s' is already open; ignoring this caller's different parameters",
        port.c_str());
    }
    return bus;
  }

  std::shared_ptr<CanBus> instance;
  try {
    instance = loader().createSharedInstance(type);
  } catch (const pluginlib::PluginlibException & e) {
    throw std::runtime_error("can't load transport plugin '" + type + "': " + e.what());
  }
  instance->open(port, params);

  // Close the device as soon as the last holder lets go, then drop the plugin instance.
  std::shared_ptr<CanBus> bus(
    instance.get(), [instance](CanBus * b) mutable {
      b->close();
      instance.reset();
    });
  entry = Entry{type, params, bus};
  RCLCPP_INFO(logger(), "opened %s with %s", port.c_str(), type.c_str());
  return bus;
}

}  // namespace hw_transport
