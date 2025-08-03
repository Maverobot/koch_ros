#include "koch_hardware/koch_hardware.h"
#include <chrono>
#include <thread>

namespace koch_hardware
{

hardware_interface::CallbackReturn KochHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  leader_arm_ = std::make_shared<dynamixel_hardware::DynamixelHardware>();
  follower_arm_ = std::make_shared<dynamixel_hardware::DynamixelHardware>();
  leader_arm_->on_init(info);
  follower_arm_->on_init(info);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KochHardware::export_state_interfaces()
{
  auto si1 = leader_arm_->export_state_interfaces();
  auto si2 = follower_arm_->export_state_interfaces();

  // Move si2's content into si1 (move semantics)
  si1.reserve(si1.size() + si2.size());
  for (auto & si : si2)
  {
    si1.push_back(std::move(si));
  }
  return si1;
}

std::vector<hardware_interface::CommandInterface> KochHardware::export_command_interfaces()
{
  auto ci1 = leader_arm_->export_command_interfaces();
  auto ci2 = follower_arm_->export_command_interfaces();

  // Move ci2's content into ci1 (move semantics)
  ci1.reserve(ci1.size() + ci2.size());
  for (auto & ci : ci2)
  {
    ci1.push_back(std::move(ci));
  }
  return ci1;  // Return by move
}

hardware_interface::return_type KochHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto f1 = std::async(std::launch::async, [&]() { return leader_arm_->read(time, period); });
  auto f2 = std::async(std::launch::async, [&]() { return follower_arm_->read(time, period); });
  return (f1.get() == hardware_interface::return_type::OK &&
          f2.get() == hardware_interface::return_type::OK)
           ? hardware_interface::return_type::OK
           : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type KochHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto f1 = std::async(std::launch::async, [&]() { return leader_arm_->write(time, period); });
  auto f2 = std::async(std::launch::async, [&]() { return follower_arm_->write(time, period); });
  return (f1.get() == hardware_interface::return_type::OK &&
          f2.get() == hardware_interface::return_type::OK)
           ? hardware_interface::return_type::OK
           : hardware_interface::return_type::ERROR;
}
}  // namespace koch_hardware
