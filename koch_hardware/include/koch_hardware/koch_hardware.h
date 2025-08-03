#pragma once

#include <future>
#include <memory>
#include <vector>
#include "dynamixel_hardware/dynamixel_hardware.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace koch_hardware
{

class KochHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::shared_ptr<dynamixel_hardware::DynamixelHardware> leader_arm_;
  std::shared_ptr<dynamixel_hardware::DynamixelHardware> follower_arm_;
};

}  // namespace koch_hardware
