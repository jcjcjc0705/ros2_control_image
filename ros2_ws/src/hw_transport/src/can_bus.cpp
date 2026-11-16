#include "hw_transport/can_bus.hpp"

#include <cctype>
#include <cstdio>
#include <string>

namespace hw_transport
{

namespace
{
int hex_value(char c)
{
  if (c >= '0' && c <= '9') {return c - '0';}
  c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  if (c >= 'a' && c <= 'f') {return c - 'a' + 10;}
  return -1;
}
}  // namespace

bool parse_can_frame(const std::string & text, CanFrame & frame)
{
  const auto hash = text.find('#');
  if (hash == std::string::npos || hash == 0) {return false;}

  const std::string id_text = text.substr(0, hash);
  if (id_text.size() != 3 && id_text.size() != 8) {return false;}
  std::uint32_t id = 0;
  for (char c : id_text) {
    const int v = hex_value(c);
    if (v < 0) {return false;}
    id = (id << 4) | static_cast<std::uint32_t>(v);
  }

  CanFrame result;
  result.extended = id_text.size() == 8;
  if (result.extended ? id > 0x1FFFFFFF : id > 0x7FF) {return false;}
  result.id = id;

  const std::string data_text = text.substr(hash + 1);
  if (data_text == "R" || data_text == "r") {
    result.remote = true;
  } else {
    std::string hex;
    for (char c : data_text) {
      if (c != '.') {hex += c;}  // cansend allows 11.22.33
    }
    if (hex.size() % 2 != 0 || hex.size() > 16) {return false;}
    for (std::size_t i = 0; i < hex.size(); i += 2) {
      const int hi = hex_value(hex[i]);
      const int lo = hex_value(hex[i + 1]);
      if (hi < 0 || lo < 0) {return false;}
      result.data[result.len++] = static_cast<std::uint8_t>((hi << 4) | lo);
    }
  }
  frame = result;
  return true;
}

std::string to_string(const CanFrame & frame)
{
  char buffer[64];
  int n = std::snprintf(
    buffer, sizeof(buffer), frame.extended ? "%08X" : "%03X", static_cast<unsigned>(frame.id));
  n += std::snprintf(buffer + n, sizeof(buffer) - n, "   [%u] ", frame.len);
  if (frame.remote) {
    std::snprintf(buffer + n, sizeof(buffer) - n, " remote request");
  } else {
    for (std::uint8_t i = 0; i < frame.len && i < 8; ++i) {
      n += std::snprintf(buffer + n, sizeof(buffer) - n, " %02X", frame.data[i]);
    }
  }
  return buffer;
}

bool CanBus::send(const std::vector<CanFrame> & frames)
{
  bool ok = true;
  for (const auto & frame : frames) {
    ok = send(frame) && ok;
  }
  return ok;
}

CanBus::SubscriptionId CanBus::subscribe(std::uint32_t id, bool extended, Callback callback)
{
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  const SubscriptionId sub = next_subscription_id_++;
  subscriptions_.push_back({sub, false, id, extended, std::move(callback)});
  return sub;
}

CanBus::SubscriptionId CanBus::subscribe_all(Callback callback)
{
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  const SubscriptionId sub = next_subscription_id_++;
  subscriptions_.push_back({sub, true, 0, false, std::move(callback)});
  return sub;
}

void CanBus::unsubscribe(SubscriptionId subscription)
{
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  for (auto it = subscriptions_.begin(); it != subscriptions_.end(); ++it) {
    if (it->id == subscription) {
      subscriptions_.erase(it);
      return;
    }
  }
}

void CanBus::dispatch(const CanFrame & frame)
{
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  for (const auto & sub : subscriptions_) {
    if (sub.all || (sub.can_id == frame.id && sub.extended == frame.extended)) {
      sub.callback(frame);
    }
  }
}

}  // namespace hw_transport
