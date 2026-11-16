#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace hw_transport
{

/// One classic CAN 2.0 frame (up to 8 data bytes).
struct CanFrame
{
  std::uint32_t id = 0;   ///< 11-bit standard or 29-bit extended identifier
  bool extended = false;  ///< true: `id` is a 29-bit extended identifier
  bool remote = false;    ///< remote transmission request
  std::uint8_t len = 0;   ///< number of valid bytes in `data` (0-8)
  std::array<std::uint8_t, 8> data{};
};

/// Parses cansend syntax: "123#0102AA" (standard), "1234ABCD#01" (8 hex digits: extended),
/// "123#R" (remote). Returns false if `text` isn't a valid frame.
bool parse_can_frame(const std::string & text, CanFrame & frame);

/// Formats like candump: "123   [3]  01 02 AA".
std::string to_string(const CanFrame & frame);

/// Free-form settings for a transport, from URDF <param> tags or the command line.
using Params = std::unordered_map<std::string, std::string>;

/**
 * A CAN bus, as seen by hardware components. Each development board (USB-CAN adapter,
 * SocketCAN interface, ...) implements this as a pluginlib plugin, so a motor's hardware
 * component never depends on which board it is wired to.
 *
 * Get one with acquire_can_bus() (registry.hpp), not by constructing it: that lets several
 * hardware components (e.g. joint motors and a wheel driver on the same adapter) share it.
 *
 * Implementations must:
 *  - open(): open the device and start a receive thread that calls dispatch() for every
 *    received frame;
 *  - send(): queue the frame and return without waiting for any reply, so a hardware
 *    component can send one command per device in write() and collect replies in read();
 *  - be safe to call send() from several threads.
 */
class CanBus
{
public:
  using Callback = std::function<void (const CanFrame &)>;
  using SubscriptionId = std::uint64_t;

  virtual ~CanBus() = default;

  /// Opens `port` (e.g. "can0", "/dev/can_lf"). Throws std::runtime_error on failure.
  virtual void open(const std::string & port, const Params & params) = 0;
  /// Stops the receive thread and closes the device. Safe to call more than once.
  virtual void close() = 0;
  virtual bool is_open() const = 0;

  /// Sends one frame without waiting. Returns false if it couldn't be queued.
  virtual bool send(const CanFrame & frame) = 0;
  /// Sends several frames. Adapters that can pack frames into one transfer override this.
  /// Returns false if any frame couldn't be queued.
  virtual bool send(const std::vector<CanFrame> & frames);

  /// Calls `callback` from the receive thread for every frame with this identifier.
  /// Keep callbacks short (copy the data out); they run while holding the subscription
  /// lock, so they must not call subscribe()/unsubscribe().
  SubscriptionId subscribe(std::uint32_t id, bool extended, Callback callback);
  /// Calls `callback` for every received frame (debug tools).
  SubscriptionId subscribe_all(Callback callback);
  void unsubscribe(SubscriptionId subscription);

protected:
  /// Implementations call this from their receive thread for every received frame.
  void dispatch(const CanFrame & frame);

private:
  struct Subscription
  {
    SubscriptionId id;
    bool all;
    std::uint32_t can_id;
    bool extended;
    Callback callback;
  };

  std::mutex subscriptions_mutex_;
  std::vector<Subscription> subscriptions_;
  SubscriptionId next_subscription_id_ = 1;
};

}  // namespace hw_transport
