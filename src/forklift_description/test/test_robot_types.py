"""
Static validation of the robot types: standard, high_lift (differential) and mecanum.

Each type is the same xacro with an overlay file (config/forklift_high_lift.yaml,
config/forklift_mecanum.yaml). The tests check that each type has the capabilities that it
is made for (lift range, speed, capacity), that the geometry is consistent (mast, stability,
lift actuator), and that the existing mission can be done with each type.
"""

import pathlib

from forklift_description.urdf_kinematics import generate_urdf, Model
import numpy as np
import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
SRC = PKG.parent
XACRO = PKG / 'urdf' / 'forklift.urdf.xacro'
BASE = yaml.safe_load((PKG / 'config' / 'forklift_params.yaml').read_text())
OVERLAYS = {'standard': None, 'high_lift': PKG / 'config' / 'forklift_high_lift.yaml',
            'mecanum': PKG / 'config' / 'forklift_mecanum.yaml'}
# Top-level keys of an overlay that are not sections of the base file.
EXTRA_KEYS = {'drive_type', 'colors', 'mecanum', 'navigation', 'navigation_footprints',
              'lift_controller', 'load_handler'}
G = 9.81


def params_of(robot_type):
    p = dict(BASE)
    if OVERLAYS[robot_type]:
        p.update(yaml.safe_load(OVERLAYS[robot_type].read_text()))
    return p


def model_of(robot_type):
    kw = {'sim_backend': 'mock'}
    if OVERLAYS[robot_type]:
        kw['params_overlay'] = OVERLAYS[robot_type]
    return Model(generate_urdf(XACRO, **kw))


def max_fork_top(p):
    return p['lift']['min_height'] + p['lift']['max_travel']


def braking_decel(p):
    """Return the braking limit of the navigation of this robot type (m/s^2)."""
    smoother = p.get('navigation', {}).get('velocity_smoother')
    return abs(smoother['max_decel'][0]) if smoother else 0.5


def mission():
    path = SRC / 'forklift_task_manager' / 'config' / 'warehouse_mission.yaml'
    if not path.exists():
        pytest.skip('mission file not found')
    return yaml.safe_load(path.read_text())


def place_clearance():
    path = SRC / 'forklift_control' / 'config' / 'load_handler.yaml'
    if not path.exists():
        pytest.skip('load_handler.yaml not found')
    return yaml.safe_load(path.read_text())['load_handler']['ros__parameters']['place_clearance']


# ----------------------------------------------------------------------------- structure

@pytest.mark.parametrize('robot_type', ['high_lift', 'mecanum'])
def test_overlay_replaces_only_known_sections(robot_type):
    overlay = yaml.safe_load(OVERLAYS[robot_type].read_text())
    unknown = set(overlay) - set(BASE) - EXTRA_KEYS
    assert not unknown, f'unknown top-level keys {unknown}'
    for section, value in overlay.items():
        if section in BASE and isinstance(BASE[section], dict):
            # The replacement is per section: a missing key would be missing in the URDF.
            assert set(BASE[section]) <= set(value) | {'caster'}, \
                f'section {section} lacks keys {set(BASE[section]) - set(value)}'


@pytest.mark.parametrize('robot_type', sorted(OVERLAYS))
def test_every_robot_type_generates_a_urdf(robot_type):
    model = model_of(robot_type)
    assert 'fork_lift_joint' in model.joints
    upper = float(model.joints['fork_lift_joint'].find('limit').get('upper'))
    assert upper == pytest.approx(params_of(robot_type)['lift']['max_travel'])


# ----------------------------------------------------------------------------- capability

def test_the_high_lift_robot_reaches_two_metres():
    assert max_fork_top(params_of('high_lift')) >= 2.0


def test_lift_range_order_high_lift_standard_mecanum():
    tops = {t: max_fork_top(params_of(t)) for t in OVERLAYS}
    assert tops['high_lift'] > tops['standard'] > tops['mecanum']
    assert tops['mecanum'] <= 0.6, 'the Mecanum robot lifts a pallet only a little'


def test_capacity_order_high_lift_standard_mecanum():
    load = {t: params_of(t)['lift']['rated_load'] for t in OVERLAYS}
    assert load['high_lift'] >= load['standard'] > load['mecanum']


def test_the_mecanum_robot_is_faster_than_the_others():
    mecanum = params_of('mecanum')
    standard_rim = BASE['drive_wheel']['max_velocity'] * BASE['drive_wheel']['radius']
    mecanum_rim = mecanum['drive_wheel']['max_velocity'] * mecanum['drive_wheel']['radius']
    assert mecanum_rim > 1.2 * standard_rim
    nav = mecanum['navigation']
    standard_nav = yaml.safe_load(
        (SRC / 'forklift_navigation' / 'config' / 'nav2_params.yaml').read_text())
    standard_speed = standard_nav['controller_server']['ros__parameters']['FollowPath'][
        'desired_linear_vel']
    assert nav['controller_server']['FollowPath']['desired_linear_vel'] > standard_speed
    assert nav['velocity_smoother']['max_velocity'][0] > standard_speed


def test_the_high_lift_robot_is_not_faster_than_the_standard_robot():
    overlay = yaml.safe_load(OVERLAYS['high_lift'].read_text())
    # It does not change the drive or the navigation limits: the speed is the standard speed.
    assert 'drive_wheel' not in overlay and 'navigation' not in overlay
    assert overlay.get('drive_type', 'diff') == 'diff'


@pytest.mark.parametrize('robot_type', ['high_lift', 'mecanum'])
def test_the_lift_profile_is_inside_the_hard_limits(robot_type):
    p = params_of(robot_type)
    profile = p['lift_controller']['lift_controller']
    assert 0 < profile['default_velocity'] <= p['lift']['max_velocity']


# ----------------------------------------------------------------------------- geometry

@pytest.mark.parametrize('robot_type', sorted(OVERLAYS))
def test_the_mast_is_taller_than_the_highest_carriage_bar(robot_type):
    p = params_of(robot_type)
    top_bar = max_fork_top(p) + p['carriage']['upper_bar_z'] + p['carriage']['bar_height'] / 2
    assert p['mast']['height'] > top_bar, 'the carriage must not leave the mast'
    assert p['mast']['height'] - top_bar < 0.6, 'the mast must not be much taller than necessary'


def test_the_mast_height_order_makes_the_types_visibly_different():
    heights = {t: params_of(t)['mast']['height'] for t in OVERLAYS}
    assert heights['high_lift'] > heights['standard'] + 0.5
    assert heights['mecanum'] < heights['standard'] - 0.5


@pytest.mark.parametrize('robot_type', ['high_lift', 'mecanum'])
def test_lift_actuator_can_hold_carriage_forks_and_rated_load(robot_type):
    p = params_of(robot_type)
    moving = p['carriage']['mass'] + 2 * p['forks']['mass']
    assert p['lift']['max_effort'] > 1.5 * (moving + p['lift']['rated_load']) * G


@pytest.mark.parametrize('robot_type', sorted(OVERLAYS))
@pytest.mark.parametrize('level', ['low', 'max'])
def test_rated_load_does_not_tip_forward_when_braking(robot_type, level):
    """The COM must stay behind the front axle when the robot brakes with its rated load."""
    p = params_of(robot_type)
    q = 0.20 if level == 'low' else p['lift']['max_travel']
    model = model_of(robot_type)
    total, com = model.mass_and_com({'fork_lift_joint': q})
    load = p['lift']['rated_load']
    load_pos = model.point('fork_carriage_link', np.array([p['forks']['length'] / 2, 0, 0.3]),
                           {'fork_lift_joint': q})
    com = (total * com + load * load_pos) / (total + load)
    shifted_x = com[0] + com[2] * braking_decel(p) / G
    assert shifted_x < -0.03, f'tipping margin only {-shifted_x:.3f} m'


@pytest.mark.parametrize('robot_type', sorted(OVERLAYS))
def test_centre_of_mass_is_inside_the_support_polygon_at_every_height(robot_type):
    p = params_of(robot_type)
    model = model_of(robot_type)
    # The support polygon: from the rear contacts (x = -1.00) to the front axle (x = 0).
    for q in (0.0, p['lift']['max_travel'] / 2, p['lift']['max_travel']):
        _, com = model.mass_and_com({'fork_lift_joint': q})
        assert -1.0 + 0.15 < com[0] < -0.15, f'COM x={com[0]:.3f} at q={q}'
        assert abs(com[1]) < 1e-3


# ----------------------------------------------------------------------------- mission

@pytest.mark.parametrize('robot_type', sorted(OVERLAYS))
def test_each_type_can_place_a_load_on_every_mission_platform(robot_type):
    """The existing mission must be possible with each robot type (the lift range is enough)."""
    m = mission()
    p = params_of(robot_type)
    pocket = m['load']['pallet']['height'] - m['load']['pallet']['deck_thickness']
    for name, dest in m['destinations'].items():
        needed_q = dest['surface_height'] + place_clearance() + pocket - p['lift']['min_height']
        assert needed_q < p['lift']['max_travel'] - 0.02, \
            f'{robot_type} cannot reach {name}: needs q={needed_q:.2f}'


@pytest.mark.parametrize('robot_type', sorted(OVERLAYS))
def test_each_type_can_carry_the_mission_load(robot_type):
    m = mission()
    assert m['load']['mass'] < params_of(robot_type)['lift']['rated_load']
