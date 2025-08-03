cmake_minimum_required(VERSION 3.10)
project(koch_hardware)

find_package(ament_cmake REQUIRED)
find_package(dynamixel_hardware REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/koch_hardware.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  dynamixel_hardware
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${dynamixel_hardware_INCLUDE_DIRS}
)

pluginlib_export_plugin_description_file(hardware_interface koch_hardware.xml)

ament_export_include_directories(include)
ament_export_libraries(koch_hardware)
ament_export_dependencies(
  dynamixel_hardware
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
