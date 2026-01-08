# koch_ros

ROS 2 packages for the Koch v1.1 robot arm, a low-cost 6-DOF manipulator with a gripper.

## Packages

- **koch_description**: URDF/Xacro robot description, meshes, and visualization configs
- **koch_controllers**: Custom ros2_control controllers for the robot

## Controllers

### LeaderFollowerController
Teleoperation controller that mirrors a leader arm's joint positions to a follower arm.

```sh
ros2 launch koch_controllers leader_follower.launch.py
```

### ClawMachineController
![Alt Text](./demo.gif)

Cartesian velocity controller for the follower arm with gripper control:
- Subscribes to `/cmd_vel` (geometry_msgs/TwistStamped) for end-effector velocity commands
- Subscribes to `/grasping` (std_msgs/Bool) to open/close the gripper
- Uses inverse kinematics with nullspace optimization to minimize end-effector rotation

```sh
ros2 launch koch_controllers claw_machine.launch.py
```

I made a teleop device using a ESP32-S3 board with a joystick and button to publish the `/cmd_vel` and `/grasping` commands. See the [picoros_playground](https://github.com/Maverobot/picoros_playground).
