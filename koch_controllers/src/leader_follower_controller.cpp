#include "koch_controllers/leader_follower_controller.h"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace koch_controllers {

controller_interface::CallbackReturn LeaderFollowerController::on_init() {
  auto_declare<std::vector<std::string>>("leader_joints", {});
  auto_declare<std::vector<std::string>>("follower_joints", {});
  auto_declare<std::vector<double>>("leader_to_follower_offset", {});
  auto_declare<std::vector<double>>("leader_to_follower_scale", {});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LeaderFollowerController::on_configure(const rclcpp_lifecycle::State &) {
  leader_joint_names_ =
      get_node()->get_parameter("leader_joints").as_string_array();
  follower_joint_names_ =
      get_node()->get_parameter("follower_joints").as_string_array();

  if (leader_joint_names_.size() != follower_joint_names_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Leader/follower joint count mismatch.");
    return controller_interface::CallbackReturn::ERROR;
  }

  leader_to_follower_offset_ = this->get_node()
                                   ->get_parameter("leader_to_follower_offset")
                                   .as_double_array();

  if (leader_to_follower_offset_.size() != leader_joint_names_.size()) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "leader_to_follower_offset size does not match "
                 "leader_joint_names size");
    return controller_interface::CallbackReturn::ERROR;
  }

  leader_to_follower_scale_ = this->get_node()
                                  ->get_parameter("leader_to_follower_scale")
                                  .as_double_array();

  if (leader_to_follower_scale_.size() != leader_joint_names_.size()) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "leader_to_follower_scale size does not match "
                 "leader_joint_names size");
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LeaderFollowerController::on_activate(const rclcpp_lifecycle::State &) {
  leader_state_handles_.clear();
  follower_command_handles_.clear();

  for (const auto &joint : leader_joint_names_) {
    auto handle =
        std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                     [&](const auto &iface) {
                       return iface.get_name() == joint + "/position" &&
                              iface.get_interface_name() == "position";
                     });

    RCLCPP_INFO(get_node()->get_logger(), "Available state interfaces:");
    for (const auto &iface : state_interfaces_) {
      RCLCPP_INFO(get_node()->get_logger(), " - joint: %s, interface: %s",
                  iface.get_name().c_str(), iface.get_interface_name().c_str());
    }

    if (handle == state_interfaces_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Missing state interface for joint '%s'", joint.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    leader_state_handles_.emplace_back(*handle);
  }

  for (const auto &joint : follower_joint_names_) {
    auto handle =
        std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                     [&](const auto &iface) {
                       return iface.get_name() == joint + "/position" &&
                              iface.get_interface_name() == "position";
                     });

    if (handle == command_interfaces_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Missing command interface for joint '%s'", joint.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    follower_command_handles_.emplace_back(*handle);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
LeaderFollowerController::update(const rclcpp::Time &,
                                 const rclcpp::Duration &) {
  for (size_t i = 0; i < leader_state_handles_.size(); ++i) {
    const double leader_pos =
        leader_state_handles_[i].get().get_optional().value();
    double corrected_leader_pos = leader_pos * leader_to_follower_scale_[i] +
                                  leader_to_follower_offset_[i];

    // Normalize angle if needed (e.g., keep within [-pi, pi])
    while (corrected_leader_pos > M_PI) {
      corrected_leader_pos -= 2.0 * M_PI;
    }
    while (corrected_leader_pos < -M_PI) {
      corrected_leader_pos += 2.0 * M_PI;
    }

    const bool success =
        follower_command_handles_[i].get().set_value(corrected_leader_pos);
    if (!success) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to set command for follower joint '%s' from leader "
                   "joint '%s' ",
                   follower_joint_names_[i].c_str(),
                   leader_joint_names_[i].c_str());
      return controller_interface::return_type::ERROR;
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
LeaderFollowerController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : follower_joint_names_) {
    config.names.push_back(joint + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
LeaderFollowerController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : leader_joint_names_) {
    config.names.push_back(joint + "/position");
  }
  return config;
}

} // namespace koch_controllers

PLUGINLIB_EXPORT_CLASS(koch_controllers::LeaderFollowerController,
                       controller_interface::ControllerInterface)
