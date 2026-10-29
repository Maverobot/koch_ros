# Copyright 2026 Zheng Qu
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Exercise the installed official driver against Koch URDF on ptys only."""

import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from dxl_reply_checks import assert_transient_reply
from dxl_sim import DynamixelBus
from geometry_msgs.msg import TwistStamped
import numpy as np
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint
import xacro
import yaml


TRAJECTORY_CONTROLLERS = ('joint_trajectory_controller', 'gripper_trajectory_controller')


@pytest.fixture(autouse=True)
def local_runtime_files(tmp_path, monkeypatch):
    """Keep ROS logs and launch-generated parameters inside the test artifacts."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    monkeypatch.setenv('TMPDIR', str(tmp_path))


def wait_for(predicate, timeout=15):
    """Wait for an observable condition rather than assuming startup duration."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError('Timed out waiting for simulator/driver event')


class DriverSession:
    """Run real controller manager and RSP, with verified simulator ports."""

    def __init__(self, directory, follower, leader=None, *, custom=False, launch=None,
                 hardware='robotis'):
        self.directory = directory
        self.processes = []
        self.logs = []
        self.messages = []
        self.clients = {}
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('koch_bus_test', context=self.context)
        self.executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.subscription = self.node.create_subscription(
            JointState, '/joint_states', self.messages.append, 100)
        assert 'ROS_DOMAIN_ID' in os.environ, 'Run through ROS-isolated test runner'
        assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
        buses = {'follower': follower}
        if leader is not None:
            buses['leader'] = leader
        for bus in buses.values():
            assert bus.port.startswith('/dev/pts/')
            assert os.ttyname(bus.slave) == bus.port
        share = Path(get_package_share_directory('koch_description'))
        koch_launch = launch == ('koch_description', 'koch.launch.py')
        model = 'koch.xacro' if leader or koch_launch else 'koch_follower_standalone.xacro'
        mappings = {'hardware': hardware}
        if koch_launch and leader is None:
            mappings['leader'] = 'false'
        mappings.update({f'{arm}_port': bus.port for arm, bus in buses.items()})
        urdf = xacro.process_file(str(share / 'urdf' / model), mappings=mappings).toxml()
        port_param = 'usb_port' if hardware == 'legacy' else 'port_name'
        ports = ET.fromstring(urdf).findall(
            f'./ros2_control/hardware/param[@name="{port_param}"]')
        if hardware != 'mock':
            assert {p.text for p in ports} == {b.port for b in buses.values()}
        if launch:
            package, filename = launch
            self.start('ros2cli', 'ros2', [
                'launch', package, filename, 'rviz:=false',
                *[f'{key}:={value}' for key, value in mappings.items()]])
            return
        params = directory / 'description.yaml'
        params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
            'robot_description': urdf}}}))
        self.start('robot_state_publisher', 'robot_state_publisher', [
            '--ros-args', '--params-file', str(params)])
        try:
            config = (Path(get_package_share_directory('koch_controllers'))
                      / 'config' / 'koch_controllers.yaml' if custom
                      else share / 'config' / 'controllers.yaml')
            self.manager = self.start('controller_manager', 'ros2_control_node', [
                '--ros-args', '--params-file', str(config)])
            self.assert_overlay_sdk()
        except Exception:
            self.close()
            raise

    def assert_overlay_sdk(self):
        """Fail unless this process mapped the SDK from the Koch workspace overlay."""
        expected = (Path(get_package_prefix('koch_controllers')).parent
                    / 'dynamixel_sdk/lib/libdynamixel_sdk.so').resolve()
        assert expected.is_file(), f'Build the patched SDK overlay first: {expected}'
        maps_path = Path(f'/proc/{self.manager.pid}/maps')
        wait_for(lambda: self.manager.poll() is not None
                 or 'libdynamixel_sdk.so' in maps_path.read_text())
        assert self.manager.poll() is None, self.text()
        mappings = maps_path.read_text()
        libraries = {Path(line.split()[-1]).resolve() for line in mappings.splitlines()
                     if 'libdynamixel_sdk.so' in line}
        assert libraries == {expected}, f'Wrong SDK mapped: {libraries}; expected {expected}'
        (self.directory / 'sdk-maps.txt').write_text(mappings)

    def start(self, package, executable, args):
        """Start a tracked process without shell expansion or a ROS daemon."""
        prefix = Path(get_package_prefix(package))
        path = (prefix / 'bin' / executable if package == 'ros2cli'
                else prefix / 'lib' / package / executable)
        log = open(self.directory / f'{executable}.log', 'w')
        self.logs.append(log)
        process = subprocess.Popen(
            [str(path), *args], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append(process)
        return process

    def wait(self, predicate, timeout=15):
        """Spin subscriptions and wait for an observable event."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError(self.text())

    def service(self, service_type, name):
        """Query controller manager without invoking the ROS CLI daemon."""
        if name not in self.clients:
            self.clients[name] = self.node.create_client(
                service_type, '/controller_manager/' + name)
        client = self.clients[name]
        assert client.wait_for_service(timeout_sec=10), self.text()
        future = client.call_async(service_type.Request())
        self.wait(future.done, timeout=15)
        return future.result()

    def hardware(self):
        """Read authoritative hardware lifecycle states."""
        return {h.name: h.state.label for h in self.service(
            ListHardwareComponents, 'list_hardware_components').component}

    def controllers(self):
        """Read authoritative controller lifecycle states."""
        return {c.name: c.state for c in self.service(
            ListControllers, 'list_controllers').controller}

    def spawn(self, *names):
        """Load the real Koch configured controllers and await activation."""
        process = self.start('controller_manager', 'spawner', [
            *names, '-c', '/controller_manager', '--controller-manager-timeout', '10'])
        self.wait(lambda: process.poll() is not None)
        assert process.returncode == 0, (self.directory / 'spawner.log').read_text()
        states = self.controllers()
        assert all(states.get(name) == 'active' for name in names), states

    def text(self):
        """Read the controller-manager diagnostic log."""
        path = self.directory / 'ros2_control_node.log'
        return (path if path.exists() else self.directory / 'ros2.log').read_text()

    def close(self):
        """Terminate every process started, escalating if graceful shutdown stalls."""
        for process in reversed(self.processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        for log in self.logs:
            log.close()
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()


def follower_gain_registers(servo_id):
    """Return the validated gain RAM settings replayed from e1c2142."""
    gains = {84: [1280, 1280, 2400, 1200, 1200, 1200][servo_id - 1]}
    if servo_id >= 3:
        gains[80] = 600
    return gains


@pytest.mark.parametrize('torque_on_id', [1, 6])
def test_robotis_refuses_torque_on_without_writes(tmp_path, torque_on_id):
    """Refusal preserves torque/goals; earlier servos may get mode writes."""
    bus = DynamixelBus(torque_on=(torque_on_id,))
    session = None
    try:
        session = DriverSession(tmp_path, bus)
        wait_for(lambda: 'Torque is enabled, cannot proceed' in session.text())
        wait_for(lambda: 'Failed to initialize' in session.text()
                 or session.processes[-1].poll() is not None)
        assert 'Dynamixel Hardware Start!' not in session.text()
        assert 'Successfully activated controller' not in session.text()
        # Failed resource initialization prevents manager services starting.
        assert "Successful 'activate'" not in session.text()
        assert 'Starting Controller Manager services' not in session.text()
        # Upstream 1.5.1 checks torque then initializes each GPIO, not the
        # whole bus first. Earlier torque-off servos can receive mode writes.
        for write in bus.writes:
            assert write['id'] < torque_on_id
            allowed = {11: 3, **follower_gain_registers(write['id'])}
            assert write['address'] in allowed
            assert int.from_bytes(bytes(write['data']), 'little') == allowed[write['address']]
            assert write['torque'] == 0 and write['error'] == 0
        assert len(bus.writes) == sum(
            1 + len(follower_gain_registers(i)) for i in range(1, torque_on_id))
    finally:
        if session:
            session.close()
        (tmp_path / 'bus-writes.json').write_text(json.dumps(bus.writes, indent=2))
        (tmp_path / 'bus-instructions.json').write_text(json.dumps(bus.instructions, indent=2))
        bus.close()


def test_robotis_startup_goal_before_torque(tmp_path):
    """Activation must seed measured goals before enabling any torque."""
    bus = DynamixelBus()
    session = None
    try:
        session = DriverSession(tmp_path, bus)
        wait_for(lambda: 'Dynamixel Hardware Start!' in session.text()
                 or session.processes[-1].poll() is not None)
        assert 'Dynamixel Hardware Start!' in session.text(), session.text()
        session.spawn('joint_state_broadcaster', *TRAJECTORY_CONTROLLERS)
        session.wait(lambda: len(session.messages) >= 5)
        assert session.hardware()['koch_follower_ros2_control'] == 'active'
        message = session.messages[-1]
        for i, suffix in enumerate(['1', '2', '3', '4', '5', '_gripper'], 1):
            index = message.name.index('follower_joint' + suffix)
            raw = 2048 + 10 * (i - 1)
            assert message.position[index] == pytest.approx(
                raw * 0.0015339807878856412 - 3.14159265359, abs=1e-9)
            # Workbench used float(0.229 rpm/count * 0.104719755 rad/s/rpm).
            assert message.velocity[index] == pytest.approx(-i * 0.229 * 0.104719755,
                                                            rel=1e-6)
            assert message.effort[index] == pytest.approx(-i * 2.69)
        eeprom = [w for w in bus.writes if w['address'] < 64]
        assert [(w['id'], w['address'], w['data'], w['torque']) for w in eeprom] == [
            (i, 11, [3], 0) for i in range(1, 7)]
        for i in range(1, 7):
            writes = [w for w in bus.writes if w['id'] == i]
            goal = next(w for w in writes if w['resolved'] == [116, 117, 118, 119])
            enable = next(w for w in writes if w['address'] == 64 and w['data'] == [1])
            assert goal['time'] < enable['time']
            gain_writes = [w for w in writes if w['address'] in (80, 82, 84)]
            assert {w['address']: int.from_bytes(bytes(w['data']), 'little')
                    for w in gain_writes} == follower_gain_registers(i)
            assert len(gain_writes) == len(follower_gain_registers(i))
            assert all(w['time'] < goal['time'] and w['torque'] == 0 for w in gain_writes)
            assert int.from_bytes(bytes(goal['data']), 'little') == 2048 + 10 * (i - 1)
    finally:
        if session:
            session.close()
        (tmp_path / 'bus-writes.json').write_text(json.dumps(bus.writes, indent=2))
        (tmp_path / 'bus-instructions.json').write_text(json.dumps(bus.instructions, indent=2))
        bus.close()


def test_robotis_state_only_leader(tmp_path):
    """State-only leader must activate without any torque-on or goal writes."""
    follower = DynamixelBus()
    leader = DynamixelBus()
    session = None
    try:
        session = DriverSession(tmp_path, follower, leader)
        wait_for(lambda: session.text().count('Dynamixel Hardware Start!') == 2
                 or session.processes[-1].poll() is not None)
        assert session.text().count('Dynamixel Hardware Start!') == 2, session.text()
        session.spawn('joint_state_broadcaster')
        session.wait(lambda: any('leader_joint1' in msg.name for msg in session.messages))
        assert session.hardware()['koch_leader_ros2_control'] == 'active'
        assert not any(w['address'] == 64 and w['data'] == [1] for w in leader.writes)
        assert not any(116 in w['resolved'] for w in leader.writes)
    finally:
        if session:
            session.close()
        (tmp_path / 'leader-writes.json').write_text(json.dumps(leader.writes, indent=2))
        (tmp_path / 'leader-instructions.json').write_text(
            json.dumps(leader.instructions, indent=2))
        follower.close()
        leader.close()


def assert_only_reported_positions(session, bus):
    """Reject zero-filled or otherwise invented position values."""
    for i, suffix in enumerate(['1', '2', '3', '4', '5', '_gripper'], 1):
        address = 634 if i <= 2 else 214
        reported = {int.from_bytes(data[:4], 'little', signed=True)
                    for _, servo_id, addr, data in bus.reads
                    if servo_id == i and addr == address and len(data) == 11}
        assert reported and 0 not in reported
        for msg in session.messages:
            if 'follower_joint' + suffix not in msg.name:
                continue
            pos = msg.position[msg.name.index('follower_joint' + suffix)]
            raw = (pos + 3.14159265359) / 0.0015339807878856412
            assert any(abs(raw - value) < 1e-5 for value in reported), (i, pos, reported)


def test_robotis_read_faults_hold_state_then_deactivate(tmp_path):
    """Sustained dropped reads retain real samples and stop controllers."""
    bus = DynamixelBus()
    session = None
    try:
        session = DriverSession(tmp_path, bus)
        session.spawn('joint_state_broadcaster', *TRAJECTORY_CONTROLLERS)
        session.wait(lambda: len(session.messages) >= 5)
        start = time.monotonic()
        bus.drop_forever = True
        session.wait(lambda: session.hardware()['koch_follower_ros2_control'] != 'active')
        elapsed = time.monotonic() - start
        # One 80 Hz period can straddle the injection boundary. The driver
        # accumulates manager periods, not a separate wall-clock deadline.
        assert elapsed >= 0.48, elapsed
        durations = re.findall(r'Dynamixel Read Fail \(Duration: ([0-9.]+)ms/500ms\)',
                               session.text())
        assert durations and float(durations[-1]) >= 500
        failed_hardware = session.hardware()
        failed_controllers = session.controllers()
        assert all(failed_controllers[name] != 'active' for name in TRAJECTORY_CONTROLLERS)
        assert not any(w['address'] == 64 and w['data'] == [0] for w in bus.writes)
        assert_only_reported_positions(session, bus)
        bus.drop_forever = False
        restored = time.monotonic()
        session.wait(lambda: time.monotonic() - restored >= 0.5)
        assert session.hardware()['koch_follower_ros2_control'] != 'active'
        assert all(session.controllers()[name] != 'active' for name in TRAJECTORY_CONTROLLERS)
        assert_only_reported_positions(session, bus)
        (tmp_path / 'failure-observation.json').write_text(json.dumps({
            'elapsed_seconds': elapsed, 'accumulated_failure_ms': float(durations[-1]),
            'failed_hardware': failed_hardware, 'failed_controllers': failed_controllers,
            'restored_hardware': session.hardware(), 'restored_controllers': session.controllers(),
            'torque_after_restore': [bus.registers[i][64] for i in range(1, 7)],
        }, indent=2))
    finally:
        if session:
            session.close()
        (tmp_path / 'bus-writes.json').write_text(json.dumps(bus.writes, indent=2))
        bus.close()


def test_robotis_fast_bulk_read_fallback(tmp_path):
    """Unsupported Fast Bulk Read must fall back within the 500 ms budget."""
    bus = DynamixelBus(fast_read=False)
    session = None
    try:
        session = DriverSession(tmp_path, bus)
        wait_for(lambda: 'Dynamixel Hardware Start!' in session.text()
                 or session.processes[-1].poll() is not None)
        assert 'Dynamixel Hardware Start!' in session.text(), session.text()
        fast = [t for t, _, instruction, _ in bus.instructions if instruction == 0x9a]
        normal = [t for t, _, instruction, _ in bus.instructions if instruction == 0x92]
        enable = [w['time'] for w in bus.writes if w['address'] == 64 and w['data'] == [1]]
        assert len(fast) == 10 and normal and enable
        elapsed = max(enable) - fast[0]
        (tmp_path / 'fallback-seconds.txt').write_text(str(elapsed))
        assert elapsed < 0.500
        assert session.hardware()['koch_follower_ros2_control'] == 'active'
    finally:
        if session:
            session.close()
        (tmp_path / 'bus-instructions.json').write_text(json.dumps(bus.instructions, indent=2))
        (tmp_path / 'bus-writes.json').write_text(json.dumps(bus.writes, indent=2))
        bus.close()


def goals(bus):
    """Read goal registers in radians using the pinned model conversion."""
    return [int.from_bytes(bus.registers[i][116:120], 'little', signed=True)
            * 0.0015339807878856412 - 3.14159265359 for i in range(1, 7)]


def test_leader_follower_controller_offsets_and_scales(tmp_path):
    """Verify LeaderFollowerController scale, offset and remainder(2*pi)."""
    follower = DynamixelBus()
    leader = DynamixelBus()
    session = None
    try:
        session = DriverSession(tmp_path, follower, leader, custom=True)
        session.spawn('joint_state_broadcaster', 'leader_follower_controller')
        config = yaml.safe_load((Path(get_package_share_directory('koch_controllers'))
                                 / 'config' / 'koch_controllers.yaml').read_text())
        params = config['leader_follower_controller']['ros__parameters']
        # Keep both configured pi offsets away from the wrap boundary.
        for raw in ([1800, 2200, 2400, 1900, 1700, 2100],
                    [1750, 2250, 2350, 1950, 1650, 2150]):
            for i, value in enumerate(raw, 1):
                leader.set_register(i, 132, 4, value)
            expected = [math.remainder(
                (value * 0.0015339807878856412 - 3.14159265359) * scale + offset,
                2 * math.pi) for value, scale, offset in zip(
                    raw, params['leader_to_follower_scale'], params['leader_to_follower_offset'])]
            session.wait(lambda: np.allclose(goals(follower), expected, atol=0.0016))
        assert session.controllers()['leader_follower_controller'] == 'active'
        assert not any(64 in w['resolved'] or 116 in w['resolved'] for w in leader.writes)
    finally:
        if session:
            session.close()
        follower.close()
        leader.close()


def gripper_tip(q):
    """Independent FK oracle using kinematics.h screw axes and reference pose."""
    axes = [(0, 0, 1), (-1, 0, 0), (1, 0, 0), (1, 0, 0), (0, 0, 1), (0, 1, 0)]
    points = [(0, 0, 0.039), (0, 0, 0.0563), (0, 0.014791, 0.164647),
              (0, 0.014791, 0.255114), (0, 0.014791, 0.300114),
              (0, 0.014791, 0.313264)]
    transform = np.eye(4)
    for index, (axis, point, theta) in enumerate(zip(axes, points, q)):
        theta -= math.pi / 2 if index == 2 else 0
        x, y, z = axis
        skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        rotation = np.eye(3) + math.sin(theta) * skew + (1 - math.cos(theta)) * skew @ skew
        step = np.eye(4)
        step[:3, :3] = rotation
        step[:3, 3] = (np.eye(3) - rotation) @ point
        transform = transform @ step
    return (transform @ [0, 0.014791, 0.365264, 1])[:3]


def test_claw_machine_initial_pose_motion_and_grasping(tmp_path):
    """Verify ClawMachineController maps +cmd_vel.x to +space vy and Bool to grip."""
    bus = DynamixelBus()
    session = None
    try:
        session = DriverSession(tmp_path, bus, custom=True)
        session.spawn('joint_state_broadcaster', 'claw_machine_controller')
        config = yaml.safe_load((Path(get_package_share_directory('koch_controllers'))
                                 / 'config' / 'koch_controllers.yaml').read_text())
        initial = config['claw_machine_controller']['ros__parameters']['initial_joint_positions']
        # on_activate resets grasping=false; the first update overrides joint 6
        # with the controller's default open position, not initial_joint_positions[5].
        session.wait(lambda: np.allclose(goals(bus), [*initial[:5], 1.46], atol=0.0016))
        grasp = session.node.create_publisher(Bool, '/grasping', 10)
        twist = session.node.create_publisher(TwistStamped, '/cmd_vel', 10)
        session.wait(lambda: grasp.get_subscription_count() and twist.get_subscription_count())
        grasp.publish(Bool(data=True))
        session.wait(lambda: abs(goals(bus)[5] + 0.11) < 0.0016)
        grasp.publish(Bool(data=False))
        session.wait(lambda: abs(goals(bus)[5] - 1.46) < 0.0016)
        before = gripper_tip(goals(bus))
        start = time.monotonic()

        def moving_positive_y():
            command = TwistStamped()
            command.header.stamp = session.node.get_clock().now().to_msg()
            command.twist.linear.x = 0.02
            twist.publish(command)
            return time.monotonic() - start > 0.5

        session.wait(moving_positive_y)
        displacement = gripper_tip(goals(bus)) - before
        assert displacement[1] > 0.001, displacement
        assert session.controllers()['claw_machine_controller'] == 'active'
    finally:
        if session:
            session.close()
        bus.close()


@pytest.mark.parametrize('package,filename,controller,dual', [
    ('koch_description', 'koch.launch.py', 'joint_trajectory_controller', True),
    ('koch_description', 'koch.launch.py', 'joint_trajectory_controller', False),
    ('koch_controllers', 'leader_follower.launch.py', 'leader_follower_controller', True),
    ('koch_controllers', 'claw_machine.launch.py', 'claw_machine_controller', False),
])
def test_mock_launches(tmp_path, package, filename, controller, dual):
    """All shipped control launches must activate with GenericSystem only."""
    follower = DynamixelBus()
    leader = DynamixelBus() if dual else None
    session = None
    try:
        session = DriverSession(tmp_path, follower, leader, launch=(package, filename),
                                hardware='mock')
        # Do not contend with the launch's parallel spawners during startup.
        expected_count = 3 if controller == 'joint_trajectory_controller' else 2
        session.wait(lambda: session.text().count('Configured and activated') >= expected_count,
                     timeout=40)
        assert session.controllers().get(controller) == 'active'
        session.wait(lambda: len(session.messages) >= 5)
        assert all(state == 'active' for state in session.hardware().values())
        if controller == 'joint_trajectory_controller':
            assert session.controllers()['gripper_trajectory_controller'] == 'active'
            assert_independent_trajectory_goals(session)
        assert not follower.instructions
        assert leader is None or not leader.instructions
    finally:
        if session:
            session.close()
        follower.close()
        if leader:
            leader.close()


def assert_independent_trajectory_goals(session):
    """Check split action endpoints and the live desired-state parameter."""
    parameter_client = session.node.create_client(
        GetParameters, '/joint_trajectory_controller/get_parameters')
    assert parameter_client.wait_for_service(timeout_sec=5)
    request = GetParameters.Request(names=['interpolate_from_desired_state'])
    parameter_future = parameter_client.call_async(request)
    session.wait(parameter_future.done)
    assert parameter_future.result().values[0].bool_value
    session.node.destroy_client(parameter_client)
    clients = []
    futures = []
    targets = [([f'follower_joint{i}' for i in range(1, 6)], [0.1, -0.2, 0.3, -1.5, -1.4]),
               (['follower_joint_gripper'], [0.4])]
    try:
        for name, (joints, positions) in zip(TRAJECTORY_CONTROLLERS, targets):
            client = ActionClient(session.node, FollowJointTrajectory,
                                  f'/{name}/follow_joint_trajectory')
            clients.append(client)
            assert client.wait_for_server(timeout_sec=5)
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = joints
            point = JointTrajectoryPoint(positions=positions)
            point.time_from_start.sec = 1
            goal.trajectory.points = [point]
            futures.append(client.send_goal_async(goal))
        session.wait(lambda: all(f.done() for f in futures))
        handles = [f.result() for f in futures]
        assert all(h.accepted for h in handles)
        results = [h.get_result_async() for h in handles]
        session.wait(lambda: all(f.done() for f in results))
        assert all(f.result().status == 4 for f in results)  # STATUS_SUCCEEDED
        assert all(f.result().result.error_code == 0 for f in results)
        expected = {j: value for joints, values in targets for j, value in zip(joints, values)}
        session.wait(lambda: all(abs(session.messages[-1].position[
            session.messages[-1].name.index(j)] - value) < 1e-6 for j, value in expected.items()))
    finally:
        for client in clients:
            client.destroy()


@pytest.fixture
def bus_session(tmp_path):
    """Own a simulator/session and retain wire artifacts even on assertion failure."""
    buses, sessions = [], []

    def start(bus, hardware='robotis', launch=None):
        buses.append(bus)
        session = DriverSession(tmp_path, bus, hardware=hardware, launch=launch)
        sessions.append(session)
        return session

    yield start
    for session in reversed(sessions):
        session.close()
    for bus in buses:
        (tmp_path / 'bus-writes.json').write_text(json.dumps(bus.writes, indent=2))
        (tmp_path / 'bus-instructions.json').write_text(json.dumps(bus.instructions, indent=2))
        (tmp_path / 'fault-events.json').write_text(json.dumps(bus.fault_events, indent=2))
        (tmp_path / 'group-transactions.json').write_text(
            json.dumps(bus.group_transactions, indent=2))
        bus.close()


def test_koch_launch_robotis_without_leader(bus_session):
    """The follower-only launch must activate all three trajectory-stack controllers."""
    bus = DynamixelBus()
    session = bus_session(bus, launch=('koch_description', 'koch.launch.py'))
    session.wait(lambda: session.text().count('Configured and activated') >= 3, timeout=40)
    assert session.hardware() == {'koch_follower_ros2_control': 'active'}
    states = session.controllers()
    assert all(states.get(name) == 'active'
               for name in ('joint_state_broadcaster', *TRAJECTORY_CONTROLLERS))
    assert bus.instructions


def follower_positions(message):
    """Select joint positions rather than the additional raw GPIO state entries."""
    return {name: position for name, position in zip(message.name, message.position)
            if name.startswith('follower_joint')}


def assert_held_positions(session, first, expected):
    """Check every published sample since injection, not only the final sample."""
    assert len(session.messages) > first
    for message in session.messages[first:]:
        assert follower_positions(message) == pytest.approx(expected, abs=1e-9)


@pytest.mark.parametrize('fast', [True, False], ids=['fast_bulk', 'bulk'])
@pytest.mark.parametrize('fault', ['drop', 'crc', 'silent'])
def test_robotis_transient_fault_is_observed(bus_session, fast, fault):
    """Every injected fault reaches the driver; both read modes retain valid states."""
    bus = DynamixelBus(fast_read=fast)
    session = bus_session(bus)
    session.spawn('joint_state_broadcaster', *TRAJECTORY_CONTROLLERS)
    session.wait(lambda: len(session.messages) >= 5)
    first = len(session.messages)
    expected = follower_positions(session.messages[-1])
    window_start = len(bus.group_transactions) - 2
    assert window_start >= 0
    if fault == 'drop':
        bus.drop_cycles = 1
    elif fault == 'crc':
        bus.corrupt_cycles = 1
    else:
        bus.silent_cycles[3] = 1
    session.wait(lambda: any(tx['faults'] for tx in bus.group_transactions[window_start:]))
    injected_indices = [index for index, tx in enumerate(bus.group_transactions)
                        if any(kind == fault and mode == fast for _, kind, mode in tx['faults'])]
    assert len(injected_indices) == 1
    index = injected_indices[0]
    session.wait(lambda: len(bus.group_transactions) >= index + 4)
    transactions = bus.group_transactions
    assert_transient_reply(fault, fast, bytes.fromhex(transactions[index - 1]['reply']),
                           bytes.fromhex(transactions[index]['reply']))
    # The driver logs after receiving this reply and before requesting the next
    # group. Require exactly that failure, with no unrelated failures in the
    # two-clean-reads-before / two-clean-reads-after window.
    failures = [float(stamp) for stamp in re.findall(
        r'\[([0-9]+\.[0-9]+)\] \[dynamixel_hardware_interface\]: '
        r'Dynamixel Read Fail \(Duration:', session.text())]
    in_window = [stamp for stamp in failures
                 if transactions[window_start]['received_at'] <= stamp
                 < transactions[index + 3]['received_at']]
    assert len(in_window) == 1, f'Expected exactly one injected read failure: {in_window}'
    assert (transactions[index]['received_at'] <= in_window[0]
            < transactions[index + 1]['received_at'])
    session.wait(lambda: len(session.messages) >= first + 5)
    assert_held_positions(session, first, expected)
    assert_only_reported_positions(session, bus)
    assert session.hardware()['koch_follower_ros2_control'] == 'active'
    assert all(session.controllers()[name] == 'active' for name in TRAJECTORY_CONTROLLERS)


def test_robotis_error_only_bulk_reply_holds_then_times_out(bus_session):
    """CRC-valid error status without data must count as a failed ordinary read."""
    bus = DynamixelBus(fast_read=False)
    session = bus_session(bus)
    session.spawn('joint_state_broadcaster', *TRAJECTORY_CONTROLLERS)
    session.wait(lambda: len(session.messages) >= 5)
    first = len(session.messages)
    expected = follower_positions(session.messages[-1])
    bus.error_status_ids.add(3)
    session.wait(lambda: 'Dynamixel Read Fail (Duration:' in session.text())
    assert session.hardware()['koch_follower_ros2_control'] == 'active'
    session.wait(lambda: session.hardware()['koch_follower_ros2_control'] != 'active')
    durations = re.findall(r'Dynamixel Read Fail \(Duration: ([0-9.]+)ms/500ms\)', session.text())
    assert float(durations[-1]) >= 500
    assert any(kind == 'error_status' for _, kind, _ in bus.fault_events)
    assert_held_positions(session, first, expected)
    assert all(session.controllers()[name] != 'active' for name in TRAJECTORY_CONTROLLERS)
    assert not any(w['address'] == 64 and w['data'] == [0] for w in bus.writes)


def test_robotis_short_first_fast_reply_never_seeds_fake_goal(bus_session):
    """Reject a CRC-valid truncated aggregate before seeding or enabling torque."""
    bus = DynamixelBus()
    bus.short_fast_cycles = 1
    session = bus_session(bus)
    session.wait(lambda: 'Dynamixel Hardware Start!' in session.text())
    assert len([e for e in bus.fault_events if e[1] == 'short_fast']) == 1
    assert 'FastBulkRead TxRx failed (attempt 1/10)' in session.text()
    assert bus.valid_group_replies
    for write in bus.writes:
        if 116 in write['resolved']:
            raw = int.from_bytes(bytes(write['data']), 'little')
            assert raw == 2048 + 10 * (write['id'] - 1)
            assert write['time'] > bus.valid_group_replies[0]
        if write['address'] == 64 and write['data'] == [1]:
            assert write['time'] > bus.valid_group_replies[0]
    assert session.hardware()['koch_follower_ros2_control'] == 'active'


def test_robotis_short_runtime_fast_reply_holds_state(bus_session):
    """A truncated Fast aggregate cannot replace a previously valid state."""
    bus = DynamixelBus()
    session = bus_session(bus)
    session.spawn('joint_state_broadcaster', *TRAJECTORY_CONTROLLERS)
    session.wait(lambda: len(session.messages) >= 5)
    first = len(session.messages)
    expected = follower_positions(session.messages[-1])
    bus.short_fast_cycles = 1
    session.wait(lambda: 'Dynamixel Read Fail (Duration:' in session.text())
    session.wait(lambda: len(session.messages) >= first + 5)
    assert any(kind == 'short_fast' for _, kind, _ in bus.fault_events)
    assert_held_positions(session, first, expected)
    assert session.hardware()['koch_follower_ros2_control'] == 'active'


@pytest.mark.parametrize('fast', [True, False], ids=['fast_bulk', 'bulk'])
def test_robotis_alert_only_keeps_runtime_data_valid(bus_session, fast):
    """An Alert-only status preserves data and remains diagnostic after activation."""
    bus = DynamixelBus(fast_read=fast)
    session = bus_session(bus)
    session.spawn('joint_state_broadcaster', *TRAJECTORY_CONTROLLERS)
    session.wait(lambda: len(session.messages) >= 5)
    failures = session.text().count('Dynamixel Read Fail (Duration:')
    bus.set_register(3, 70, 1, 4)
    bus.set_register(3, 126, 2, -123)
    session.wait(lambda: session.messages[-1].effort[
        session.messages[-1].name.index('follower_joint3')] == pytest.approx(-123 * 2.69))
    assert session.text().count('Dynamixel Read Fail (Duration:') == failures
    assert session.hardware()['koch_follower_ros2_control'] == 'active'
    assert all(session.controllers()[name] == 'active' for name in TRAJECTORY_CONTROLLERS)


def test_robotis_ping_alert_reboots_before_torque_refusal(bus_session):
    """Pin the upstream hazard: ping Alert reboots an already torque-on joint."""
    bus = DynamixelBus(torque_on=(3,))
    bus.set_register(3, 70, 1, 4)
    session = bus_session(bus)
    session.wait(lambda: 'Dynamixel Hardware Start!' in session.text())
    assert len(bus.reboots) == 1
    reboot_time, servo_id, torque = bus.reboots[0]
    assert (servo_id, torque) == (3, 1)
    torque_reads = [t for t, i, addr, _ in bus.reads if i == 3 and addr == 64]
    assert torque_reads and min(torque_reads) > reboot_time
    assert 'Torque is enabled, cannot proceed' not in session.text()
    assert session.hardware()['koch_follower_ros2_control'] == 'active'


def test_legacy_error_only_sync_reply_holds_state(bus_session):
    """Verify fixed legacy driver plus overlay SDK rejects error-only Sync replies."""
    from ament_index_python.packages import PackageNotFoundError
    try:
        get_package_prefix('dynamixel_hardware')
    except PackageNotFoundError:
        pytest.skip('Optional fixed legacy driver underlay is not sourced')
    bus = DynamixelBus()
    # Legacy enables torque before its first read. Give the simulated servo a
    # matching goal at rest so this check isolates read validation, not seeding.
    for i in range(1, 7):
        bus.set_register(i, 116, 4, 2048 + 10 * (i - 1))
    session = bus_session(bus, hardware='legacy')
    session.spawn('joint_state_broadcaster', *TRAJECTORY_CONTROLLERS)
    session.wait(lambda: len(session.messages) >= 5)
    first = len(session.messages)
    expected = follower_positions(session.messages[-1])
    bus.error_status_ids.add(3)
    session.wait(lambda: 'read failures for' in session.text())
    assert any(instruction == 0x82 for _, _, instruction, _ in bus.instructions)
    session.wait(lambda: len(session.messages) >= first + 3)
    assert_held_positions(session, first, expected)
    bus.error_status_ids.clear()
    session.wait(lambda: 'Communication recovered after read failures' in session.text())
    assert session.hardware()['koch_follower_ros2_control'] == 'active'


def test_robotis_hardware_error_at_activation_refuses_until_timeout(bus_session):
    """Valid Alert data is not sufficient for startup while Hardware Error is set."""
    bus = DynamixelBus()
    # Introduce the error after ping, avoiding the separate ping/reboot path.
    bus.hardware_errors_at_group_read[3] = 4
    session = bus_session(bus)
    session.wait(lambda: 'Dynamixel Start Fail (Timeout:' in session.text())
    duration = re.search(r'Dynamixel Start Fail \(Timeout: ([0-9.]+)ms/500ms', session.text())
    assert duration and float(duration.group(1)) >= 500
    assert session.hardware()['koch_follower_ros2_control'] != 'active'
    assert not any(116 in w['resolved'] or w['address'] == 64 for w in bus.writes)
    assert not bus.reboots
