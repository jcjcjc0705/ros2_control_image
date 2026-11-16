#include "mit_controller/mit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace mit_controller
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

const std::array<std::string, MitController::kNumInterfaces> MitController::kInterfaces = {
  hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY, "kp", "kd",
  hardware_interface::HW_IF_EFFORT};

CallbackReturn MitController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<double>("command_timeout", 0.5);
    auto_declare<double>("damping_kd", 1.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "declaring parameters failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration MitController::command_interface_configuration() const
{
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & joint : joints_) {
    for (const auto & name : kInterfaces) {
      config.names.push_back(joint + "/" + name);
    }
  }
  return config;
}

InterfaceConfiguration MitController::state_interface_configuration() const
{
  // Only to aim the (unused, kp = 0) position target at the current position in damping.
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

CallbackReturn MitController::on_configure(const rclcpp_lifecycle::State &)
{
  const auto logger = get_node()->get_logger();
  joints_ = get_node()->get_parameter("joints").as_string_array();
  command_timeout_ = get_node()->get_parameter("command_timeout").as_double();
  damping_kd_ = get_node()->get_parameter("damping_kd").as_double();

  if (joints_.empty()) {
    RCLCPP_ERROR(logger, "parameter 'joints' is empty");
    return CallbackReturn::ERROR;
  }
  if (std::set<std::string>(joints_.begin(), joints_.end()).size() != joints_.size()) {
    RCLCPP_ERROR(logger, "parameter 'joints' lists a joint more than once");
    return CallbackReturn::ERROR;
  }
  if (!std::isfinite(damping_kd_) || damping_kd_ < 0.0) {
    RCLCPP_ERROR(logger, "parameter 'damping_kd' must be >= 0, got %f", damping_kd_);
    return CallbackReturn::ERROR;
  }

  subscription_ = get_node()->create_subscription<mit_msgs::msg::MitCommand>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const mit_msgs::msg::MitCommand::SharedPtr msg) {on_command(*msg);});

  RCLCPP_INFO(
    logger, "configured for %zu joints, command_timeout %.3f s, damping_kd %.3f",
    joints_.size(), command_timeout_, damping_kd_);
  return CallbackReturn::SUCCESS;
}

void MitController::on_command(const mit_msgs::msg::MitCommand & msg)
{
  const auto logger = get_node()->get_logger();
  auto & clock = *get_node()->get_clock();
  const std::size_t n = joints_.size();

  // order[i] = index into joints_ of the message's i-th entry
  std::vector<std::size_t> order(n);
  if (msg.joint_names.empty()) {
    for (std::size_t i = 0; i < n; ++i) {order[i] = i;}
  } else {
    if (msg.joint_names.size() != n) {
      RCLCPP_WARN_THROTTLE(
        logger, clock, 1000, "rejected command: %zu joint_names, expected %zu",
        msg.joint_names.size(), n);
      return;
    }
    std::unordered_map<std::string, std::size_t> index;
    for (std::size_t i = 0; i < n; ++i) {index[joints_[i]] = i;}
    std::vector<bool> seen(n, false);
    for (std::size_t i = 0; i < n; ++i) {
      const auto it = index.find(msg.joint_names[i]);
      if (it == index.end() || seen[it->second]) {
        RCLCPP_WARN_THROTTLE(
          logger, clock, 1000, "rejected command: unknown or repeated joint '%s'",
          msg.joint_names[i].c_str());
        return;
      }
      seen[it->second] = true;
      order[i] = it->second;
    }
  }

  const std::array<const std::vector<double> *, kNumInterfaces> fields = {
    &msg.position, &msg.velocity, &msg.kp, &msg.kd, &msg.effort};
  auto command = std::make_shared<Command>();
  for (std::size_t f = 0; f < kNumInterfaces; ++f) {
    const auto & in = *fields[f];
    if (in.size() != n) {
      RCLCPP_WARN_THROTTLE(
        logger, clock, 1000, "rejected command: '%s' has %zu values, expected %zu",
        kInterfaces[f].c_str(), in.size(), n);
      return;
    }
    auto & out = command->values[f];
    out.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
      const double v = in[i];
      if (!std::isfinite(v) || ((f == kKp || f == kKd) && v < 0.0)) {
        RCLCPP_WARN_THROTTLE(
          logger, clock, 1000, "rejected command: %s[%zu] = %f", kInterfaces[f].c_str(), i, v);
        return;
      }
      out[order[i]] = v;
    }
  }

  command->sequence = next_sequence_++;
  command_buffer_.writeFromNonRT(command);
}

CallbackReturn MitController::on_activate(const rclcpp_lifecycle::State &)
{
  const auto logger = get_node()->get_logger();
  std::unordered_map<std::string, std::size_t> joint_index;
  for (std::size_t j = 0; j < joints_.size(); ++j) {joint_index[joints_[j]] = j;}

  // Loaned interfaces aren't guaranteed to come in the requested order; index them by name.
  constexpr std::size_t kMissing = static_cast<std::size_t>(-1);
  command_index_.assign(joints_.size(), {kMissing, kMissing, kMissing, kMissing, kMissing});
  for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
    const auto & ci = command_interfaces_[i];
    const auto j = joint_index.find(ci.get_prefix_name());
    const auto f = std::find(kInterfaces.begin(), kInterfaces.end(), ci.get_interface_name());
    if (j != joint_index.end() && f != kInterfaces.end()) {
      command_index_[j->second][static_cast<std::size_t>(f - kInterfaces.begin())] = i;
    }
  }
  position_state_index_.assign(joints_.size(), kMissing);
  for (std::size_t i = 0; i < state_interfaces_.size(); ++i) {
    const auto j = joint_index.find(state_interfaces_[i].get_prefix_name());
    if (j != joint_index.end()) {position_state_index_[j->second] = i;}
  }
  for (std::size_t j = 0; j < joints_.size(); ++j) {
    for (std::size_t f = 0; f < kNumInterfaces; ++f) {
      if (command_index_[j][f] == kMissing) {
        RCLCPP_ERROR(
          logger, "missing command interface %s/%s", joints_[j].c_str(), kInterfaces[f].c_str());
        return CallbackReturn::ERROR;
      }
    }
    if (position_state_index_[j] == kMissing) {
      RCLCPP_ERROR(logger, "missing state interface %s/position", joints_[j].c_str());
      return CallbackReturn::ERROR;
    }
  }

  // Never act on a command received before this activation.
  command_buffer_.writeFromNonRT(nullptr);
  last_sequence_ = 0;
  damping_ = true;
  write_damping();
  RCLCPP_INFO(logger, "active, in damping until the first command on ~/commands");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MitController::on_deactivate(const rclcpp_lifecycle::State &)
{
  write_damping();
  return CallbackReturn::SUCCESS;
}

bool MitController::set(std::size_t joint, Field field, double value)
{
  return command_interfaces_[command_index_[joint][field]].set_value(value);
}

void MitController::write_damping()
{
  for (std::size_t j = 0; j < joints_.size(); ++j) {
    const auto q = state_interfaces_[position_state_index_[j]].get_optional();
    bool ok = set(j, kPosition, q.value_or(0.0));
    ok = set(j, kVelocity, 0.0) && ok;
    ok = set(j, kKp, 0.0) && ok;
    ok = set(j, kKd, damping_kd_) && ok;
    ok = set(j, kEffort, 0.0) && ok;
    if (!ok) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "couldn't write damping to %s", joints_[j].c_str());
    }
  }
}

controller_interface::return_type MitController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  // The buffer keeps the previous command alive, so nothing is freed in this thread.
  const std::shared_ptr<Command> & command = *command_buffer_.readFromRT();

  if (command && command->sequence != last_sequence_) {
    last_sequence_ = command->sequence;
    last_command_time_ = time;  // same clock as `time`, so the subtraction below is valid
  }
  const bool fresh = command && last_sequence_ != 0 &&
    (command_timeout_ <= 0.0 || (time - last_command_time_).seconds() <= command_timeout_);

  if (!fresh) {
    if (!damping_) {
      RCLCPP_WARN(
        get_node()->get_logger(), "no command for %.3f s, switching to damping",
        command_timeout_);
      damping_ = true;
    }
    write_damping();
    return controller_interface::return_type::OK;
  }

  if (damping_) {
    RCLCPP_INFO(get_node()->get_logger(), "receiving commands, leaving damping");
    damping_ = false;
  }
  bool ok = true;
  for (std::size_t j = 0; j < joints_.size(); ++j) {
    for (std::size_t f = 0; f < kNumInterfaces; ++f) {
      ok = set(j, static_cast<Field>(f), command->values[f][j]) && ok;
    }
  }
  if (!ok) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "some command interfaces couldn't be written this cycle");
  }
  return controller_interface::return_type::OK;
}

}  // namespace mit_controller

PLUGINLIB_EXPORT_CLASS(mit_controller::MitController, controller_interface::ControllerInterface)
