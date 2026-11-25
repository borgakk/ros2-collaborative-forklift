"""
Static validation of the Mecanum variant of the forklift URDF (drive_type: mecanum).

The Mecanum robot uses the same xacro as the differential robot. The overlay file
config/forklift_mecanum.yaml replaces the `drive_wheel` section and sets drive_type.
"""

import pathlib
import xml.etree.ElementTree as ET

from forklift_description.urdf_kinematics import generate_urdf, Model
import numpy as np
import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
XACRO = PKG / 'urdf' / 'forklift.urdf.xacro'
OVERLAY = PKG / 'config' / 'forklift_mecanum.yaml'
BASE = yaml.safe_load((PKG / 'config' / 'forklift_params.yaml').read_text())
MEC = yaml.safe_load(OVERLAY.read_text())
WHEELS = {
    'front_left_drive_wheel': (0.0, +1),
    'front_right_drive_wheel': (0.0, -1),
    'rear_left_drive_wheel': (-MEC['drive_wheel']['wheelbase'], +1),
    'rear_right_drive_wheel': (-MEC['drive_wheel']['wheelbase'], -1),
}


@pytest.fixture(scope='module')
def urdf():
    return generate_urdf(XACRO, params_overlay=OVERLAY, sim_backend='mock')


@pytest.fixture(scope='module')
def model(urdf):
    return Model(urdf)


def test_overlay_declares_mecanum_and_replaces_only_its_sections():
    assert MEC['drive_type'] == 'mecanum'
    extra = {'drive_type', 'mecanum', 'colors', 'navigation', 'navigation_footprints',
             'lift_controller'}
    assert set(MEC) <= set(BASE) | extra, 'unknown top-level section'
    # The wheel size must be the same as the front wheels of the differential robot.
    assert MEC['drive_wheel']['radius'] == pytest.approx(BASE['drive_wheel']['radius'])
    assert MEC['drive_wheel']['width'] == pytest.approx(BASE['drive_wheel']['width'])


def test_no_casters_and_four_continuous_wheel_joints(model):
    assert not [n for n in model.links if 'caster' in n]
    assert not [n for n in model.joints if 'caster' in n]
    for name in WHEELS:
        joint = model.joints[f'{name}_joint']
        assert joint.get('type') == 'continuous'
        assert joint.find('parent').get('link') == 'base_link'
        assert joint.find('child').get('link') == f'{name}_link'
        axis = [float(v) for v in joint.find('axis').get('xyz').split()]
        assert axis == [0.0, 1.0, 0.0], 'positive rotation must move the robot forward'
    assert 'left_drive_wheel_joint' not in model.joints


@pytest.mark.parametrize('name', sorted(WHEELS))
def test_wheel_positions_form_the_rectangle(model, name):
    x, sign = WHEELS[name]
    centre = model.point(f'{name}_link', np.zeros(3))
    assert centre[0] == pytest.approx(x, abs=1e-9)
    assert centre[1] == pytest.approx(sign * MEC['drive_wheel']['separation'] / 2, abs=1e-9)
    # All four wheels touch the ground (centre at one radius above base_footprint).
    assert centre[2] == pytest.approx(MEC['drive_wheel']['radius'], abs=1e-9)


@pytest.mark.parametrize('name', sorted(WHEELS))
def test_wheel_collision_is_a_sphere_and_has_mass(model, name):
    link = model.links[f'{name}_link']
    geometry = link.find('collision/geometry')[0]
    assert geometry.tag == 'sphere'
    assert float(geometry.get('radius')) == pytest.approx(MEC['drive_wheel']['radius'])
    assert float(link.find('inertial/mass').get('value')) == pytest.approx(
        MEC['drive_wheel']['mass'])


def test_centre_of_mass_is_inside_the_wheel_rectangle(model):
    _, com = model.mass_and_com()
    assert -MEC['drive_wheel']['wheelbase'] < com[0] < 0.0
    assert abs(com[1]) < MEC['drive_wheel']['separation'] / 2


def test_ros2_control_exports_the_four_wheel_velocity_interfaces(urdf):
    root = ET.fromstring(urdf)
    joints = {j.get('name'): j for j in root.findall('ros2_control/joint')}
    assert set(joints) == {f'{n}_joint' for n in WHEELS} | {'fork_lift_joint'}
    for name in WHEELS:
        cmd = joints[f'{name}_joint'].findall('command_interface')
        assert [c.get('name') for c in cmd] == ['velocity']
        limit = float(cmd[0].find("param[@name='max']").text)
        assert limit == pytest.approx(MEC['drive_wheel']['max_velocity'])


def test_the_default_robot_is_still_differential():
    model = Model(generate_urdf(XACRO, sim_backend='mock'))
    assert {'left_caster_link', 'right_caster_link'} <= set(model.links)
    assert 'front_left_drive_wheel_link' not in model.links


def test_unknown_drive_type_is_rejected(tmp_path):
    overlay = tmp_path / 'bad.yaml'
    overlay.write_text('drive_type: tricycle\n')
    with pytest.raises(Exception, match='drive_type'):
        generate_urdf(XACRO, params_overlay=overlay, sim_backend='mock')
