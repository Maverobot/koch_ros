from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory("koch_description")

    xacro_file = os.path.join(share_dir, "urdf", "koch.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_urdf}],
    )

    # Joint state publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    # # Launch Gazebo Sim (Ignition/GZ)
    gazebo_sim = ExecuteProcess(cmd=["gz", "sim", "-r", "--verbose"], output="screen")

    # Spawn robot into simulation using ros_gz_sim
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "koch",
            "-topic",
            "robot_description",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_publisher_node,
            gazebo_sim,
            spawn_entity,
        ]
    )
