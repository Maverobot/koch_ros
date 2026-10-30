cmake_minimum_required(VERSION 3.5)
project(koch_controllers)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)

# Add kinematics library
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
add_library(kinematics SHARED src/kinematics/kinematics.cpp)
target_include_directories(kinematics PUBLIC include)
target_link_libraries(kinematics PUBLIC Eigen3::Eigen)
target_link_libraries(kinematics PRIVATE urdf::urdf)

# Add leader-follower controller
add_library(${PROJECT_NAME} SHARED
  src/claw_machine_controller.cpp
  src/leader_follower_controller.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(koch_controllers PUBLIC
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  hardware_interface::mock_components
  pluginlib::pluginlib
  rclcpp::rclcpp
)
target_link_libraries(koch_controllers PRIVATE kinematics)

pluginlib_export_plugin_description_file(controller_interface koch_controllers.xml)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(TARGETS ${PROJECT_NAME} kinematics
  LIBRARY DESTINATION lib)

install(FILES
  koch_controllers.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics kinematics)
  ament_add_ros_isolated_gtest(test_controller_kinematics test/test_controller_kinematics.cpp
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/controller_test_logs)
  target_link_libraries(test_controller_kinematics ${PROJECT_NAME} kinematics)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_ros_isolated_test(test_robotis_bus
    COMMAND ${Python3_EXECUTABLE} -m pytest
      ${CMAKE_CURRENT_SOURCE_DIR}/test
      --junitxml=${CMAKE_CURRENT_BINARY_DIR}/test_results/test_robotis_bus.xml
      --basetemp=${CMAKE_CURRENT_BINARY_DIR}/test_robotis_bus
    RESULT_FILE ${CMAKE_CURRENT_BINARY_DIR}/test_results/test_robotis_bus.xml
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    TIMEOUT 180
  )
endif()

ament_package()
