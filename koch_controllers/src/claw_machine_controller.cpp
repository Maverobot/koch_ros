#include "koch_controllers/claw_machine_controller.h"
#include <koch_controllers/kinematics/kinematics.h>
#include "pseudo_inverse.h"

#include <Eigen/Dense>
#include <algorithm>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace koch_controllers
{

controller_interface::CallbackReturn ClawMachineController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<double>("cmd_vel_timeout", 0.1);
  auto_declare<std::vector<double>>("initial_joint_positions", {});
  auto_declare<double>("gripper_open_position", 1.46);
  auto_declare<double>("gripper_closed_position", -0.11);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ClawMachineController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  cmd_vel_timeout_ = get_node()->get_parameter("cmd_vel_timeout").as_double();

  joint_position_command_ = get_node()->get_parameter("initial_joint_positions").as_double_array();

  gripper_open_position_ = get_node()->get_parameter("gripper_open_position").as_double();
  gripper_closed_position_ = get_node()->get_parameter("gripper_closed_position").as_double();

  if (!joint_position_command_.empty() && joint_position_command_.size() != joint_names_.size())
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Size of 'initial_joint_positions' (%zu) does not match number of joints (%zu). "
      "Ignoring initial positions.",
      joint_position_command_.size(), joint_names_.size());
    joint_position_command_.clear();
  }

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (joint_names_.size() != 6)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected 6 joints (5 arm joints + 1 gripper joint), but got %zu.",
      joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Create /cmd_vel subscriber
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(&ClawMachineController::cmdVelCallback, this, std::placeholders::_1));

  // Create /grasping subscriber
  grasping_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "/grasping", rclcpp::SystemDefaultsQoS(),
    std::bind(&ClawMachineController::graspingCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "ClawMachineController configured. Subscribed to /cmd_vel with timeout: %.3f s. "
    "Gripper positions: open=%.3f, closed=%.3f",
    cmd_vel_timeout_, gripper_open_position_, gripper_closed_position_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ClawMachineController::on_activate(
  const rclcpp_lifecycle::State &)
{
  position_state_handles_.clear();
  velocity_state_handles_.clear();
  position_command_handles_.clear();

  for (const auto & joint : joint_names_)
  {
    {
      auto handle = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [&](const auto & iface) {
          return iface.get_name() == joint + "/position" &&
                 iface.get_interface_name() == "position";
        });

      if (handle == state_interfaces_.end())
      {
        RCLCPP_ERROR(
          get_node()->get_logger(), "Missing position state interface for joint '%s'",
          joint.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      position_state_handles_.emplace_back(*handle);
    }

    {
      auto handle = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [&](const auto & iface) {
          return iface.get_name() == joint + "/velocity" &&
                 iface.get_interface_name() == "velocity";
        });

      if (handle == state_interfaces_.end())
      {
        RCLCPP_ERROR(
          get_node()->get_logger(), "Missing velocity state interface for joint '%s'",
          joint.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      velocity_state_handles_.emplace_back(*handle);
    }

    {
      auto handle = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&](const auto & iface) {
          return iface.get_name() == joint + "/position" &&
                 iface.get_interface_name() == "position";
        });

      if (handle == command_interfaces_.end())
      {
        RCLCPP_ERROR(
          get_node()->get_logger(), "Missing position command interface for joint '%s'",
          joint.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      position_command_handles_.emplace_back(*handle);
    }
  }

  // Reset last cmd_vel info on activation
  last_cmd_vel_.reset();

  // Reset grasping state on activation
  grasping_state_ = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

void ClawMachineController::cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  last_cmd_vel_ = msg;
}

void ClawMachineController::graspingCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  grasping_state_ = msg->data;
}

controller_interface::return_type ClawMachineController::update(
  const rclcpp::Time & time, const rclcpp::Duration & duration)
{
  constexpr size_t NUM_ARM_JOINTS = 5;
  constexpr size_t GRIPPER_JOINT_INDEX = 5;
  constexpr double MAX_LINEAR_SPEED = 0.05;  // 5 cm/s in m/s

  if (joint_position_command_.empty())
  {
    joint_position_command_.reserve(joint_names_.size());
    for (const auto & handle_ref : position_state_handles_)
    {
      const auto & handle = handle_ref.get();
      joint_position_command_.push_back(handle.get_optional().value());
    }
  }

  // Control gripper based on grasping state
  joint_position_command_[GRIPPER_JOINT_INDEX] =
    grasping_state_ ? gripper_closed_position_ : gripper_open_position_;

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 1000, "Grasping state: %s, gripper: %.3f",
    grasping_state_ ? "true" : "false", joint_position_command_[GRIPPER_JOINT_INDEX]);

  const bool has_cmd_vel = (last_cmd_vel_ != nullptr);
  bool cmd_vel_stale = true;

  if (has_cmd_vel)
  {
    const rclcpp::Duration age = time - last_cmd_vel_->header.stamp;
    cmd_vel_stale = age.seconds() > cmd_vel_timeout_;
  }

  if (!has_cmd_vel || cmd_vel_stale)
  {
    RCLCPP_INFO_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "cmd_vel is stale or not received yet. Age: %.3f s (timeout: %.3f s)",
      has_cmd_vel ? (time - last_cmd_vel_->header.stamp).seconds() : -1.0, cmd_vel_timeout_);
  }
  else
  {
    const auto & cmd = last_cmd_vel_->twist;

    // Scale cmd_vel to limit maximum speed to MAX_LINEAR_SPEED
    double vx = cmd.linear.x;
    double vy = cmd.linear.y;
    double vz = cmd.linear.z;
    double speed = std::sqrt(vx * vx + vy * vy + vz * vz);

    double scale = 1.0;
    if (speed > MAX_LINEAR_SPEED)
    {
      scale = MAX_LINEAR_SPEED / speed;
    }

    double scaled_vx = vx * scale;
    double scaled_vy = vy * scale;
    double scaled_vz = vz * scale;

    RCLCPP_INFO_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "cmd_vel: linear=(%.3f, %.3f, %.3f), scaled=(%.3f, %.3f, %.3f), scale=%.3f", cmd.linear.x,
      cmd.linear.y, cmd.linear.z, scaled_vx, scaled_vy, scaled_vz, scale);

    // Use only the first 5 joints for arm IK (exclude gripper)
    Eigen::Matrix<double, 6, 1> q_full;
    for (size_t i = 0; i < 6; ++i)
    {
      q_full[i] = position_state_handles_[i].get().get_optional().value();
    }

    // Desired twist: only vx, vy and vz from cmd_vel, constrain wx, wy to zero and leave wz free
    // Full twist order in space Jacobian: [wx, wy, wz, vx, vy, vz]
    // Note that the cmd_vel linear.x corresponds to vy, and -linear.y to vx
    Eigen::Matrix<double, 5, 1> desired_twist_reduced;
    desired_twist_reduced << 0.0, 0.0, -scaled_vy, scaled_vx, scaled_vz;

    // Get full 6x6 Jacobian, then extract 5x5 for arm joints only
    Eigen::Matrix<double, 6, 6> Js_full = spaceJacobian(q_full);

    // Extract rows for constrained twist components and columns for arm joints only
    Eigen::Matrix<double, 5, 5> Js_arm_reduced;
    Js_arm_reduced.row(0) = Js_full.row(0).head<NUM_ARM_JOINTS>();  // wx
    Js_arm_reduced.row(1) = Js_full.row(1).head<NUM_ARM_JOINTS>();  // wy
    Js_arm_reduced.row(2) = Js_full.row(3).head<NUM_ARM_JOINTS>();  // vx
    Js_arm_reduced.row(3) = Js_full.row(4).head<NUM_ARM_JOINTS>();  // vy
    Js_arm_reduced.row(4) = Js_full.row(5).head<NUM_ARM_JOINTS>();  // vz

    // Use damped pseudo-inverse for arm joints
    Eigen::MatrixXd Js_arm_reduced_inverse;
    pseudoInverse(Js_arm_reduced, Js_arm_reduced_inverse, true);

    Eigen::Matrix<double, 5, 1> dq_arm_primary = Js_arm_reduced_inverse * desired_twist_reduced;

    // Nullspace projector for arm joints
    Eigen::Matrix<double, 5, 5> N_arm =
      Eigen::Matrix<double, 5, 5>::Identity() - Js_arm_reduced_inverse * Js_arm_reduced;

    // Secondary task: minimize wz (row 2 of full Jacobian, arm joints only)
    Eigen::Matrix<double, 1, 5> Jz_arm = Js_full.row(2).head<NUM_ARM_JOINTS>();
    double wz_primary = (Jz_arm * dq_arm_primary)(0);

    // Find nullspace velocity that cancels wz_primary
    Eigen::Matrix<double, 1, 5> Jz_null = Jz_arm * N_arm;
    Eigen::MatrixXd Jz_null_pinv;
    pseudoInverse(Jz_null, Jz_null_pinv, true, 0.01);
    Eigen::Matrix<double, 5, 1> dq_arm_nullspace = Jz_null_pinv * (-wz_primary);

    // Combined command for arm joints
    Eigen::Matrix<double, 5, 1> dq_arm_cmd = dq_arm_primary + N_arm * dq_arm_nullspace;

    // Update arm joint positions only
    for (size_t i = 0; i < NUM_ARM_JOINTS; ++i)
    {
      joint_position_command_[i] +=
        dq_arm_cmd[i] * duration.seconds();  // Integrate velocity to position
    }
  }

  for (size_t i = 0; i < position_command_handles_.size(); ++i)
  {
    const bool success = position_command_handles_[i].get().set_value(joint_position_command_[i]);
    if (!success)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to set command for joint '%s'", joint_names_[i].c_str());
      return controller_interface::return_type::ERROR;
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
ClawMachineController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_)
  {
    config.names.push_back(joint + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration ClawMachineController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_)
  {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

}  // namespace koch_controllers

PLUGINLIB_EXPORT_CLASS(
  koch_controllers::ClawMachineController, controller_interface::ControllerInterface)
