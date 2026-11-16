#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "mit_msgs/msg/mit_command.hpp"

namespace mit_controller
{

/**
 * Forwards MitCommand messages (~/commands) to each joint's five MIT command interfaces:
 * position, velocity, kp, kd, effort.
 *
 * Until the first command arrives, and whenever no new command has arrived for
 * `command_timeout` seconds, it puts every joint in damping: kp = 0, kd = damping_kd,
 * velocity = 0, effort = 0, position = the measured position. The same happens when the
 * controller is deactivated.
 *
 * Parameters:
 *   joints           string[]  joints to command (required)
 *   command_timeout  double    seconds without a new command before damping; <= 0 disables (0.5)
 *   damping_kd       double    kd used in damping, N*m*s/rad (1.0)
 */
class MitController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Command interface names per joint, in this order.
  static constexpr std::size_t kNumInterfaces = 5;
  static const std::array<std::string, kNumInterfaces> kInterfaces;

private:
  enum Field : std::size_t { kPosition = 0, kVelocity, kKp, kKd, kEffort };

  /// A validated command, reordered to match `joints_`.
  struct Command
  {
    std::uint64_t sequence = 0;
    std::array<std::vector<double>, kNumInterfaces> values;
  };

  void on_command(const mit_msgs::msg::MitCommand & msg);
  void write_damping();
  bool set(std::size_t joint, Field field, double value);

  std::vector<std::string> joints_;
  double command_timeout_ = 0.5;
  double damping_kd_ = 1.0;

  rclcpp::Subscription<mit_msgs::msg::MitCommand>::SharedPtr subscription_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<Command>> command_buffer_;
  std::uint64_t next_sequence_ = 1;  // written only by the subscription callback

  // Index into command_interfaces_ / state_interfaces_, per joint (and field).
  std::vector<std::array<std::size_t, kNumInterfaces>> command_index_;
  std::vector<std::size_t> position_state_index_;

  // update()-only state.
  std::uint64_t last_sequence_ = 0;
  rclcpp::Time last_command_time_;
  bool damping_ = true;
};

}  // namespace mit_controller
