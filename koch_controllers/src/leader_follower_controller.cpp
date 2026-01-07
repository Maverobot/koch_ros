#include <koch_controllers/kinematics/kinematics.h>
#include <koch_controllers/leader_follower_controller.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace koch_controllers
{

controller_interface::CallbackReturn LeaderFollowerController::on_init()
{
  auto_declare<std::vector<std::string>>("leader_joints", {});
  auto_declare<std::vector<std::string>>("follower_joints", {});
  auto_declare<std::vector<double>>("leader_to_follower_offset", {});
  auto_declare<std::vector<double>>("leader_to_follower_scale", {});

  twist_pub_ = get_node()->create_publisher<geometry_msgs::msg::Twist>("end_effector_twist", 10);
  pose_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("end_effector_pose", 10);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeaderFollowerController::on_configure(
  const rclcpp_lifecycle::State &)
{
  leader_joint_names_ = get_node()->get_parameter("leader_joints").as_string_array();
  follower_joint_names_ = get_node()->get_parameter("follower_joints").as_string_array();

  if (leader_joint_names_.size() != follower_joint_names_.size())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Leader/follower joint count mismatch.");
    return controller_interface::CallbackReturn::ERROR;
  }

  leader_to_follower_offset_ =
    this->get_node()->get_parameter("leader_to_follower_offset").as_double_array();

  if (leader_to_follower_offset_.size() != leader_joint_names_.size())
  {
    RCLCPP_ERROR(
      this->get_node()->get_logger(),
      "leader_to_follower_offset size does not match "
      "leader_joint_names size");
    return controller_interface::CallbackReturn::ERROR;
  }

  leader_to_follower_scale_ =
    this->get_node()->get_parameter("leader_to_follower_scale").as_double_array();

  if (leader_to_follower_scale_.size() != leader_joint_names_.size())
  {
    RCLCPP_ERROR(
      this->get_node()->get_logger(),
      "leader_to_follower_scale size does not match "
      "leader_joint_names size");
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeaderFollowerController::on_activate(
  const rclcpp_lifecycle::State &)
{
  leader_position_state_handles_.clear();
  follower_position_state_handles_.clear();
  follower_velocity_state_handles_.clear();
  follower_position_command_handles_.clear();

  for (const auto & joint : leader_joint_names_)
  {
    auto handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&](const auto & iface) {
        return iface.get_name() == joint + "/position" && iface.get_interface_name() == "position";
      });

    if (handle == state_interfaces_.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Missing position state interface for joint '%s'", joint.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    leader_position_state_handles_.emplace_back(*handle);
  }

  for (const auto & joint : follower_joint_names_)
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

      follower_position_state_handles_.emplace_back(*handle);
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

      follower_velocity_state_handles_.emplace_back(*handle);
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
          get_node()->get_logger(), "Missing positiion command interface for joint '%s'",
          joint.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      follower_position_command_handles_.emplace_back(*handle);
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LeaderFollowerController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  for (size_t i = 0; i < leader_position_state_handles_.size(); ++i)
  {
    const double leader_pos = leader_position_state_handles_[i].get().get_optional().value();
    double corrected_leader_pos =
      leader_pos * leader_to_follower_scale_[i] + leader_to_follower_offset_[i];

    // Normalize angle if needed (e.g., keep within [-pi, pi])
    corrected_leader_pos = std::remainder(corrected_leader_pos, 2.0 * M_PI);

    const bool success =
      follower_position_command_handles_[i].get().set_value(corrected_leader_pos);
    if (!success)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to set command for follower joint '%s' from leader "
        "joint '%s' ",
        follower_joint_names_[i].c_str(), leader_joint_names_[i].c_str());
      return controller_interface::return_type::ERROR;
    }
  }

  Eigen::Matrix<double, 6, 1> q;
  Eigen::Matrix<double, 6, 1> dq;
  for (size_t i = 0; i < follower_position_state_handles_.size(); ++i)
  {
    q[i] = follower_position_state_handles_[i].get().get_optional().value();
    dq[i] = follower_velocity_state_handles_[i].get().get_optional().value();
  }

  Eigen::Matrix<double, 6, 6> Js = spaceJacobian(q);
  Eigen::Matrix<double, 6, 1> twist = Js * dq;

  if (twist_pub_ && twist_pub_->get_subscription_count() > 0)
  {
    geometry_msgs::msg::Twist msg;
    msg.angular.x = twist(0);
    msg.angular.y = twist(1);
    msg.angular.z = twist(2);
    msg.linear.x = twist(3);
    msg.linear.y = twist(4);
    msg.linear.z = twist(5);
    twist_pub_->publish(msg);
  }

  // Compute and publish end-effector pose using forward kinematics
  if (pose_pub_ && pose_pub_->get_subscription_count() > 0)
  {
    Eigen::Matrix4d T_0e = forwardKinematics(q);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = time;
    pose_msg.header.frame_id = "base_link";

    pose_msg.pose.position.x = T_0e(0, 3);
    pose_msg.pose.position.y = T_0e(1, 3);
    pose_msg.pose.position.z = T_0e(2, 3);

    // Extract rotation matrix and convert to quaternion
    Eigen::Matrix3d R = T_0e.topLeftCorner<3, 3>();
    Eigen::Quaterniond q_rot(R);
    q_rot.normalize();

    pose_msg.pose.orientation.x = q_rot.x();
    pose_msg.pose.orientation.y = q_rot.y();
    pose_msg.pose.orientation.z = q_rot.z();
    pose_msg.pose.orientation.w = q_rot.w();

    pose_pub_->publish(pose_msg);
  }

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
LeaderFollowerController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : follower_joint_names_)
  {
    config.names.push_back(joint + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
LeaderFollowerController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : leader_joint_names_)
  {
    config.names.push_back(joint + "/position");
  }
  for (const auto & joint : follower_joint_names_)
  {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

}  // namespace koch_controllers

PLUGINLIB_EXPORT_CLASS(
  koch_controllers::LeaderFollowerController, controller_interface::ControllerInterface)
