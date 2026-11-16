#pragma once

#include <memory>
#include <string>

#include "hw_transport/can_bus.hpp"

namespace hw_transport
{

/**
 * Returns the CanBus for `port`, opening it with transport plugin `type`
 * (e.g. "hw_transport/SocketCan", "dm_usb2can/Usb2Can") on first use.
 *
 * Later calls with the same `port` return the same instance, so hardware components for
 * different devices on one bus share it instead of fighting over the device. The bus is
 * closed when the last holder releases it. The first caller's `params` are used; later
 * callers' params are ignored (with a warning if they differ).
 *
 * Throws std::runtime_error if the plugin can't be loaded, the port can't be opened, or
 * the port is already open with a different transport type.
 */
std::shared_ptr<CanBus> acquire_can_bus(
  const std::string & type, const std::string & port, const Params & params = {});

}  // namespace hw_transport
