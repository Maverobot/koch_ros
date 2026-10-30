"""Compare the forearm length delta, not absolute URDF/POE endpoint equality."""

import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
from test_robotis_bus import gripper_tip
import xacro


def rotation(axis, angle):
    """Compute a rotation about a unit joint axis."""
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * skew @ skew


def urdf_tip(q, robot):
    """Follow the actual URDF to its named tip, relative to follower_base_link."""
    transform = np.eye(4)
    suffixes = ['1', '2', '3', '4', '5', '_gripper', '_gripper_tip']
    for suffix, angle in zip(suffixes, [*q, 0]):
        joint = robot.find(f'./joint[@name="follower_joint{suffix}"]')
        origin = joint.find('origin')
        roll, pitch, yaw = map(float, origin.attrib['rpy'].split())
        step = np.eye(4)
        step[:3, :3] = (rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch)
                        @ rotation([1, 0, 0], roll))
        step[:3, 3] = list(map(float, origin.attrib['xyz'].split()))
        if joint.attrib['type'] != 'fixed':
            axis = list(map(float, joint.find('axis').attrib['xyz'].split()))
            step[:3, :3] = step[:3, :3] @ rotation(axis, angle)
        transform = transform @ step
    return transform[:3, 3]


@pytest.mark.parametrize('model', ['koch.xacro', 'koch_follower_standalone.xacro'])
@pytest.mark.parametrize('hardware', ['robotis', 'legacy', 'mock'])
def test_forearm_delta_matches_urdf(model, hardware, record_testsuite_property):
    """The same 10 mm displacement reaches the same tip in both approximations."""
    path = Path(get_package_share_directory('koch_description')) / 'urdf' / model
    current = ET.fromstring(xacro.process_file(str(path), mappings={
        'hardware': hardware, 'follower_port': '/render-only/follower',
        'leader_port': '/render-only/leader'}).toxml())
    frozen = copy.deepcopy(current)
    origin = frozen.find('./joint[@name="follower_joint4"]/origin')
    xyz = origin.attrib['xyz'].split()
    xyz[1] = '0.090467'
    origin.set('xyz', ' '.join(xyz))
    residuals = []
    for q in ([0, 0, math.pi / 2, 0, 0, 0], [.2, -.4, .7, -.3, .5, .1],
              [-.6, .3, 1.2, -.8, -.4, .6], [0, 0, 0, 0, 0, 0]):
        old_urdf, new_urdf = urdf_tip(q, frozen), urdf_tip(q, current)
        old_poe, new_poe = gripper_tip(q, frozen), gripper_tip(q, current)
        assert new_urdf - old_urdf == pytest.approx(new_poe - old_poe, abs=1e-12)
        assert np.linalg.norm(new_poe - old_poe) == pytest.approx(.01, abs=1e-12)
        # Record preexisting endpoint residuals without imposing exact unification.
        residuals.append([np.linalg.norm(old_poe - old_urdf),
                          np.linalg.norm(new_poe - new_urdf)])
    record_testsuite_property(f'{model}_{hardware}_urdf_poe_tip_residuals_m_old_new', repr(residuals))
