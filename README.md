# koch_ros

ROS 2 packages for the Koch v1.1 robot arm, a low-cost 6-DOF manipulator with a gripper.

## Packages

- **koch_description**: URDF/Xacro robot description, meshes, and visualization configs
- **koch_controllers**: Custom ros2_control controllers for the robot

## Hardware backend (ROS 2 Kilted)

**The patched DynamixelSDK overlay is required for real hardware.** Stock SDK
4.0.3 can accept CRC-valid error-only or truncated read replies and expose
fabricated positions; a short first Fast reply can even seed invalid goals before
torque-on. Both ROBOTIS and legacy drivers are affected. This migration uses
ROBOTIS 1.5.1 unchanged with SDK branch `koch/validate-read-replies`, commit
`35974f0` on 4.0.3. The SDK patch preserves API/ABI, version and write behavior.
Installing only the released ROBOTIS package is **not sufficient**.

With the patched SDK checkout and `koch_ros` under your workspace's `src/`:

```sh
sudo apt install ros-kilted-dynamixel-hardware-interface
source /opt/ros/kilted/setup.bash
# From the workspace root; do not build the SDK examples/custom interfaces.
colcon build --packages-select dynamixel_sdk koch_description koch_controllers
source install/setup.bash
ldd "$(ros2 pkg prefix dynamixel_hardware_interface)/lib/libdynamixel_hardware_interface.so" \
  | grep libdynamixel_sdk
```

The resolved library must be the workspace's
`install/dynamixel_sdk/lib/libdynamixel_sdk.so`, not `/opt/ros/kilted/lib/`.
Source any legacy-driver underlay **before** this overlay. After a safe launch,
verify the actual controller-manager PID with
`grep libdynamixel_sdk.so /proc/<pid>/maps`; `ldd` alone only predicts resolution.
The pty tests enforce this process-mapping check and fail if stock SDK is loaded.

All three control launches accept `hardware:=robotis|legacy|mock` (default
`robotis`), `follower_port:=/dev/ttyACM0`, and, when a leader is included,
`leader_port:=/dev/ttyACM1`. Both top-level xacros accept the same arguments.
For no-serial previews:

```sh
ros2 launch koch_description koch.launch.py hardware:=mock rviz:=false
ros2 launch koch_controllers leader_follower.launch.py hardware:=mock rviz:=false
ros2 launch koch_controllers claw_machine.launch.py hardware:=mock rviz:=false
```

`mock` uses GenericSystem with defined initial positions, not servo dynamics.
For rollback, separately install the fixed legacy `dynamixel_hardware` driver
and select `hardware:=legacy`; keep the patched SDK overlay. Never run two drivers
against one bus. Display/Gazebo launches render only, without starting this plugin.
**Kinex lab provisioner integration is separate:** it expects legacy joint/ID
schema from an argument-less render, which now produces ROBOTIS GPIOs. Update its
render arguments/parser before use; this repository does not modify that tool.

### Interfaces and units

ROBOTIS exports follower position commands and position/velocity/effort states.
The leader is state-only, with GPIO `Torque Enable=0` and no command interfaces.
Legacy retains position/velocity commands; ROBOTIS cannot switch operating mode
at runtime, and no Koch controller needs velocity commands. GPIO order matches
joint order: IDs 1–2 XL430-W250 use Present Load; IDs 3–6 XL330-M288 use Present
Current. Firmware-selected models handle the differing indirect layouts.

Position is radians (raw 2048 = zero, 4096 counts/revolution). Signed velocity
uses 0.229 rpm/count, matching legacy float math within rounding; the upstream
model multiplier is about 0.05% low. Signed effort is raw × 2.69 for both models,
including negative XL430 load. This preserves legacy values, **not Nm** or
comparable motor torque/current across models.

### Startup, faults and operator hazards

Support the follower mechanically before startup or shutdown. With Alert-free
servos, `disable_torque_at_init=false` refuses already-on torque. Checks are per
GPIO, not whole-bus preflight: earlier torque-off servos may receive Operating
Mode 3 EEPROM and gain RAM writes before a later torque-on servo causes refusal.
Nothing is then written to that servo or later servos; no torque/goal writes occur.
**Exception: upstream reboots a servo whose ping reports Alert before checking
its torque. An already torque-on joint can lose support, then initialization can
proceed.** An Alert is not a safe refusal mechanism.

Normal startup writes mode 3, follower gains and indirect mappings; no limits,
offsets or profiles. Activation reads the measured pose, seeds goals, then enables
follower torque. Leader passivity applies to normal cyclic operation only.
Do not use `hardware_components_initial_state` to start a Koch component inactive:
the unmodified driver initializes joint states to 0.0, and early failed reads can
let the broadcaster publish zeros before activation's first complete read.
The supplied launches do not use that configuration.
Claw activation subsequently commands its initial arm pose and opens the gripper.
Nonzero Hardware Error Status during activation prevents seeding/torque until
the startup timeout; after activation it is diagnostic, not an automatic stop.
Alert-only status packets still carry valid register data.

`error_timeout_ms=500` covers activation and accumulated failed-read periods.
Unsupported Fast Bulk requires ten attempts before fallback (~355 ms on the pty);
250 ms failed before fallback. The patched SDK rejects wrong lengths/IDs and
non-Alert packet errors. Brief failures hold the last valid state; sustained
failures unconfigure hardware and deactivate its controllers **without torque-off**.
Communication restoration does not automatically recover them. This is neither
an emergency stop nor a hard wall-clock guarantee. Deactivation/destruction
attempts torque-off and can drop the arm.

Each arm has `koch_follower/` or `koch_leader/` names for `dxl_state` and services
`get_dxl_data`, `set_dxl_data`, `reboot_dxl`, `set_dxl_torque`. **Raw write, torque
and reboot services bypass leader passivity and follower startup seeding.** They
can enable a leader or move/drop a follower; keep them operator-controlled with
physical support. Callbacks can block the loop; acknowledgements need not confirm
successful register writes. Runtime torque re-enable uses different ordering
from startup and can act on stale goals. Reboot does not replay all settings.
The upstream `dxl_state.torque_state` field is unreliable: do not use it as proof
of actual torque or a safety interlock. Unchecked write results and cyclic
allocation/leak concerns also remain upstream; the SDK patch does not fix them.

### Offline regression tests and bring-up

```sh
colcon test --packages-select dynamixel_sdk koch_description koch_controllers
colcon test-result --verbose
```

Tests use isolated ROS communication and simulator-owned `/dev/pts/*` only.
They cover the real plugin's startup/units, malformed ordinary/Fast replies,
Alert semantics, transient and sustained faults, LF/claw behavior and all mock
launches. The legacy Sync Read check skips cleanly without its optional underlay.
The simulator models CRC/stuffing, mixed indirect layouts, EEPROM protection and
ideal goal tracking, not USB timing, loads, power or temperature. Reboot models
only torque/error reset. Existing description copyright/flake8 failures predate
the migration; new tests add no lint failures.

Before real use: verify IDs/models/firmware and separate supply requirements;
support the mechanism and provide an independent power stop; inspect torque and
hardware errors; validate one restrained servo's units and seed-before-enable;
verify leader passivity, tiny bounded motions, supported shutdown/recovery and
sustained two-bus 80 Hz timing. Simulator success is not hardware safety approval.

## Controllers

### Follower trajectory controllers

`koch_description/launch/koch.launch.py` starts the joint state broadcaster and
two independent position trajectory controllers:

| Controller | Joints | `control_msgs/action/FollowJointTrajectory` action | `trajectory_msgs/msg/JointTrajectory` topic |
| --- | --- | --- | --- |
| `joint_trajectory_controller` | `follower_joint1`–`follower_joint5` | `/joint_trajectory_controller/follow_joint_trajectory` | `/joint_trajectory_controller/joint_trajectory` |
| `gripper_trajectory_controller` | `follower_joint_gripper` | `/gripper_trajectory_controller/follow_joint_trajectory` | `/gripper_trajectory_controller/joint_trajectory` |

Send five-joint arm trajectories separately from gripper trajectories. The arm
controller sets `interpolate_from_desired_state: true` so streamed or multi-message
references continue from the desired state rather than re-anchoring to measured
position, which left real servos with too little error to overcome friction.

Follower servo RAM gains are set at startup in `koch_hardware.xacro` (ROBOTIS)
and `koch_follower.xacro` (legacy):
P=1280 for joints 1–2 (XL430, D unchanged); P=2400/D=600 for joint 3 (XL330 elbow);
P=1200/D=600 for joints 4–5 and the gripper (XL330). Default P left a 20–30 count
dead band against gear/table friction; tuned unloaded moves settle within roughly
1–2 counts. Loaded joints still sag; the higher elbow P reduces gravity sag.
`Profile_Velocity` is intentionally unset (servo default 0). Leader tuning and
joint limits are unchanged. These controllers do not change the separate
leader-follower or claw-machine launches below.

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
