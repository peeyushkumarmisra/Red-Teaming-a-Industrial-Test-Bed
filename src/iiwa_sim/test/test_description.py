# =============================================================================
#  test_description.py - the simulated robot is the robot the IDS models
# -----------------------------------------------------------------------------
#  Expands the xacro and checks: 7 revolute joints in order; masses, gravity and mass
#  matrix at home equal the IDS's test model; the ros2_control block; plugin order;
#  the gripper; every friction profile; meshes resolve (and are not Git LFS pointer
#  files); no flange collision.
# =============================================================================

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pinocchio as pin
import pytest
import xacro
import yaml

SHARE = get_package_share_directory('iiwa_sim')
XACRO = os.path.join(SHARE, 'urdf', 'iiwa_sim.urdf.xacro')
JOINTS = [f'joint_a{i}' for i in range(1, 8)]
HOME = np.array([0.0, 0.5, 0.0, -1.2, 0.0, 0.8, 0.0])
# Reference values from the IDS's own model (ids_core/test/data/iiwa14.urdf).
GRAVITY_AT_HOME = [0.0, -26.629499422, 1.447069664, 10.876298205, 0.357804365, -0.054550086, 0.0]
MASS_DIAGONAL_AT_HOME = [1.305024634, 2.082632482, 0.436775208, 0.426462283, 0.04069311,
                         0.023771717, 0.01]
LINK_MASSES = [4.0, 4.0, 3.0, 2.7, 1.7, 1.8, 0.3 / 0.308 * 0.44]


# Run xacro with the given arguments and return the URDF text.
def expand(**mappings):
    return xacro.process_file(XACRO, mappings=mappings).toxml()


def load_yaml(name):
    with open(os.path.join(SHARE, 'config', name)) as f:
        return yaml.safe_load(f)


def plugin(root, name):
    for p in root.iter('plugin'):
        if p.get('name') == name:
            return p
    raise AssertionError(f'plugin {name} not declared')


@pytest.fixture(scope='module')
def urdf():
    return expand()


@pytest.fixture(scope='module')
def root(urdf):
    return ET.fromstring(urdf)


# The IDS needs exactly joint_a1..a7.
def test_pinocchio_sees_seven_revolute_joints_in_order(urdf):
    model = pin.buildModelFromXML(urdf)
    assert model.nq == 7
    assert list(model.names)[1:] == JOINTS


# Same masses, gravity and inertia as the IDS model.
def test_dynamics_match_the_ids_reference_model(urdf):
    model = pin.buildModelFromXML(urdf)
    data = model.createData()
    for i, mass in enumerate(LINK_MASSES):
        assert model.inertias[i + 1].mass == pytest.approx(mass, abs=1e-12)
    g = pin.computeGeneralizedGravity(model, data, HOME)
    m = pin.crba(model, data, HOME)
    m = np.triu(m) + np.triu(m, 1).T
    np.testing.assert_allclose(g, GRAVITY_AT_HOME, atol=1e-8)
    np.testing.assert_allclose(np.diag(m), MASS_DIAGONAL_AT_HOME, atol=1e-8)


# Torque control, starting at home.
def test_ros2_control_block_commands_effort_and_starts_at_home(root):
    block = root.find('ros2_control')
    assert block.find('hardware/plugin').text.strip() == 'gz_ros2_control/GazeboSimSystem'
    joints = block.findall('joint')
    assert [j.get('name') for j in joints] == JOINTS
    home = load_yaml('initial_positions.yaml')['initial_positions']
    for joint, q0 in zip(joints, home):
        assert [c.get('name') for c in joint.findall('command_interface')] == ['effort']
        states = [s.get('name') for s in joint.findall('state_interface')]
        assert states[:2] == ['position', 'velocity']
        initial = joint.find("state_interface[@name='position']/param[@name='initial_value']")
        assert float(initial.text) == pytest.approx(q0)


# Wear is applied after the controller (v1's bug).
def test_friction_plugin_is_declared_after_gz_ros2_control(root):
    names = [p.get('name') for p in root.iter('plugin')]
    assert names.index('gz_ros2_control::GazeboSimROS2ControlPlugin') < names.index(
        'iiwa_sim::FrictionDrift')
    controllers = plugin(root, 'gz_ros2_control::GazeboSimROS2ControlPlugin')
    assert controllers.find('parameters').text.endswith('config/controllers.yaml')


# The gripper attaches the payload to the flange.
def test_detachable_joint_holds_the_payload_on_link_7(root):
    p = plugin(root, 'gz::sim::systems::DetachableJoint')
    assert p.find('parent_link').text == 'link_7'
    assert p.find('child_model').text == 'payload'
    assert p.find('child_link').text == 'cube_link'
    assert p.find('attach_topic').text == '/gripper/attach'
    assert p.find('detach_topic').text == '/gripper/detach'
    assert p.find('output_topic').text == '/gripper/state'


# Each friction profile reaches the plugin.
@pytest.mark.parametrize('profile', ['none', 'nominal', 'wear', 'thesis'])
def test_friction_profiles_reach_the_plugin(profile):
    expected = load_yaml('friction.yaml')[profile]
    p = plugin(ET.fromstring(expand(friction_profile=profile)), 'iiwa_sim::FrictionDrift')
    assert float(p.find('delay').text) == pytest.approx(expected['delay'])
    joints = p.findall('joint')
    assert [j.find('name').text for j in joints] == JOINTS
    for i, j in enumerate(joints):
        assert float(j.find('initial').text) == pytest.approx(expected['initial'][i])
        assert float(j.find('rate').text) == pytest.approx(expected['rate'][i])
        assert float(j.find('max').text) == pytest.approx(expected['max'][i])


# Meshes exist and are real files, not LFS pointers.
def test_meshes_resolve_to_installed_files(root):
    meshes = [m.get('filename') for m in root.iter('mesh')]
    assert len(meshes) == 15
    for filename in meshes:
        assert filename.startswith('file://')
        path = filename[len('file://'):]
        assert os.path.isfile(path), filename
        assert os.path.getsize(path) > 1000, filename


# The held part cannot collide with the flange.
def test_flange_has_no_collision_geometry(root):
    link = root.find("link[@name='link_7']")
    assert link.find('collision') is None
    assert link.find('visual') is not None
