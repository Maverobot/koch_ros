#pragma once

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_msgs/msg/bool.hpp"

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace koch_controllers
{

class ClawMachineController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void graspingCallback(const std_msgs::msg::Bool::SharedPtr msg);

  std::vector<std::string> joint_names_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    position_state_handles_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    velocity_state_handles_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    position_command_handles_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasping_sub_;

  geometry_msgs::msg::TwistStamped::SharedPtr last_cmd_vel_;
  double cmd_vel_timeout_{0.1};  // seconds

  std::vector<double> joint_position_command_;

  bool grasping_state_{false};
  double gripper_open_position_{1.46};
  double gripper_closed_position_{-0.11};
};

}  // namespace koch_controllers
