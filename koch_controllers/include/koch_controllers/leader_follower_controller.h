#pragma once

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include <functional>

namespace koch_controllers
{

class LeaderFollowerController : public controller_interface::ControllerInterface
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
  std::vector<std::string> leader_joint_names_;
  std::vector<std::string> follower_joint_names_;
  std::vector<double> leader_to_follower_offset_;
  std::vector<double> leader_to_follower_scale_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    leader_state_handles_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    follower_command_handles_;
};

}  // namespace koch_controllers
