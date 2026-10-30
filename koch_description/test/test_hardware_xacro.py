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

"""Check hardware selection without loading a plugin or opening any device."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro


@pytest.mark.parametrize('model,arms', [
    ('koch.xacro', ['follower', 'leader']),
    ('koch_follower_standalone.xacro', ['follower']),
])
@pytest.mark.parametrize('hardware,plugin', [
    ('robotis', 'dynamixel_hardware_interface/DynamixelHardware'),
    ('legacy', 'dynamixel_hardware/DynamixelHardware'),
    ('mock', 'mock_components/GenericSystem'),
])
def test_hardware_selection(model, arms, hardware, plugin):
    """Preserve names/order and expose only the intended interfaces/backend."""
    path = Path(get_package_share_directory('koch_description')) / 'urdf' / model
    mappings = {'hardware': hardware, 'follower_port': '/render-only/follower',
                'leader_port': '/render-only/leader'}
    robot = ET.fromstring(xacro.process_file(str(path), mappings=mappings).toxml())
    for arm in arms:
        fourth = robot.find(f'./joint[@name="{arm}_joint4"]')
        xyz = [float(v) for v in fourth.find('origin').attrib['xyz'].split()]
        expected_y = 0.100467 if arm == 'follower' else 0.090467
        assert xyz == pytest.approx([0.000125, expected_y, 0.002747], abs=1e-12)
        assert fourth.find('origin').attrib['rpy'] == '1.57 0 0'
        assert fourth.find('axis').attrib['xyz'] == '1.0 0.0 0.0'
        # x is along the parallel joint axes, not part of their separation.
        expected_spacing = 0.100504547648 if arm == 'follower' else 0.090508696256
        assert math.hypot(xyz[1], xyz[2]) == pytest.approx(expected_spacing, abs=1e-9)
    systems = robot.findall('ros2_control')
    assert len(systems) == len(arms)
    names = set()
    for arm, system in zip(arms, systems):
        assert system.attrib['name'] == f'koch_{arm}_ros2_control'
        assert system.findtext('hardware/plugin') == plugin
        params = {p.attrib['name']: p.text for p in system.findall('hardware/param')}
        if hardware != 'mock':
            port = 'usb_port' if hardware == 'legacy' else 'port_name'
            assert params[port] == f'/render-only/{arm}'
        joints = system.findall('joint')
        assert [j.attrib['name'] for j in joints] == [
            f'{arm}_joint{suffix}' for suffix in ['1', '2', '3', '4', '5', '_gripper']]
        for joint in joints:
            commands = [c.attrib['name'] for c in joint.findall('command_interface')]
            expected = (['position', 'velocity'] if hardware == 'legacy'
                        else ['position'] if arm == 'follower' else [])
            assert commands == expected
            assert [s.attrib['name'] for s in joint.findall('state_interface')] == [
                'position', 'velocity', 'effort']
            if hardware == 'mock':
                assert joint.find('state_interface/param[@name="initial_value"]') is not None
        gpios = system.findall('gpio')
        if hardware == 'robotis':
            assert params['disable_torque_at_init'] == 'false'
            assert params['error_timeout_ms'] == '500'
            assert params['number_of_joints'] == params['number_of_transmissions'] == '6'
            for key in ['dynamixel_state_pub_msg_name', 'get_dynamixel_data_srv_name',
                        'set_dynamixel_data_srv_name', 'reboot_dxl_srv_name',
                        'set_dxl_torque_srv_name']:
                assert params[key].startswith(f'koch_{arm}/')
                assert params[key] not in names
                names.add(params[key])
            assert [g.findtext('param[@name="ID"]') for g in gpios] == list('123456')
            for gpio in gpios:
                assert gpio.findtext('param[@name="Operating Mode"]') == '3'
                if arm == 'leader':
                    assert gpio.findtext('param[@name="Torque Enable"]') == '0'
                    assert not gpio.findall('command_interface')
        else:
            assert not gpios
        if hardware != 'mock':
            components = joints if hardware == 'legacy' else gpios
            separator = '_' if hardware == 'legacy' else ' '
            for index, component in enumerate(components):
                p_name = separator.join(['Position', 'P', 'Gain'])
                d_name = separator.join(['Position', 'D', 'Gain'])
                p_gain = component.findtext(f'param[@name="{p_name}"]')
                d_gain = component.findtext(f'param[@name="{d_name}"]')
                if arm == 'follower':
                    assert p_gain == str([1280, 1280, 2400, 1200, 1200, 1200][index])
                    assert d_gain == ('600' if index >= 2 else None)
                else:
                    assert p_gain is None and d_gain is None


@pytest.mark.parametrize('hardware', ['robotis', 'legacy', 'mock'])
def test_koch_without_leader(hardware):
    """Omit every leader element without moving or changing the follower."""
    path = Path(get_package_share_directory('koch_description')) / 'urdf' / 'koch.xacro'
    mappings = {'hardware': hardware, 'follower_port': '/render-only/follower'}
    default = ET.fromstring(xacro.process_file(str(path), mappings=mappings).toxml())
    robot = ET.fromstring(xacro.process_file(
        str(path), mappings={**mappings, 'leader': 'false'}).toxml())
    assert 'leader_' not in ET.tostring(robot, encoding='unicode')
    assert [s.attrib['name'] for s in robot.findall('ros2_control')] == [
        'koch_follower_ros2_control']
    assert robot.find('joint[@name="follower_base_link_joint"]/origin').attrib['xyz'] == (
        '-0.2 0 0')
    assert len(robot.findall('gazebo/plugin')) == 1
    expected = [ET.tostring(element) for element in default
                if 'follower_' in ET.tostring(element, encoding='unicode')]
    actual = [ET.tostring(element) for element in robot
              if 'follower_' in ET.tostring(element, encoding='unicode')]
    assert actual == expected


def test_invalid_hardware_is_rejected():
    """Reject misspelled hardware choices rather than silently omitting a system."""
    path = Path(get_package_share_directory('koch_description')) / 'urdf' / 'koch.xacro'
    with pytest.raises(xacro.XacroException):
        xacro.process_file(str(path), mappings={'hardware': 'typo'})
