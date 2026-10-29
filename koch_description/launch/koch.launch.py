import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import xacro


def generate_launch_description():
    share_dir = get_package_share_directory("koch_description")

    xacro_file = os.path.join(share_dir, "urdf", "koch.xacro")
    rviz_config = os.path.join(share_dir, "config", "display.rviz")

    controller_config = os.path.join(share_dir, "config", "controllers.yaml")
    print(f"Loading controller config from: {controller_config}")

    # Declare a launch argument to decide whether to launch RViz
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Flag to enable RViz"
    )

    hardware_args = [
        DeclareLaunchArgument(
            'hardware', default_value='robotis',
            choices=['robotis', 'legacy', 'mock']),
        DeclareLaunchArgument('follower_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('leader_port', default_value='/dev/ttyACM1'),
    ]

    # Function to conditionally include RViz node
    def launch_setup(context, *args, **kwargs):
        robot_description_config = xacro.process_file(
            xacro_file, mappings={
                name: LaunchConfiguration(name).perform(context)
                for name in ['hardware', 'follower_port', 'leader_port']
            })
        use_rviz = LaunchConfiguration("rviz").perform(context) == "true"
        nodes = [
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    {"robot_description": robot_description_config.toxml()},
                    controller_config,
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "joint_trajectory_controller",
                    "gripper_trajectory_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description_config.toxml()}],
                output="screen",
            ),
        ]

        if use_rviz:
            nodes.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    arguments=["-d", rviz_config],
                    output="screen",
                )
            )

        return nodes

    return LaunchDescription(hardware_args + [rviz_arg, OpaqueFunction(function=launch_setup)])
