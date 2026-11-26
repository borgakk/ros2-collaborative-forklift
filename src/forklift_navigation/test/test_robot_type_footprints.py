"""
Robot types: footprints and the Nav2 overrides of robot.launch.py.

The Mecanum robot has 2 rear drive wheels instead of 2 narrow casters. Its footprint must
cover them. The `navigation` section of a robot type overlay must fit the Nav2 parameters.
"""

import importlib.util
import pathlib

from forklift_description.urdf_kinematics import generate_urdf, Model
from forklift_navigation.footprint import contains, edge_distance, load_footprints, \
    parse_nav2_string
import numpy as np
import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
SRC = PKG.parent
DESCRIPTION = SRC / 'forklift_description'
NAV2 = yaml.safe_load((PKG / 'config' / 'nav2_params.yaml').read_text())
STANDARD = load_footprints(PKG / 'config' / 'footprints.yaml')
MECANUM = load_footprints(PKG / 'config' / 'footprints_mecanum.yaml')
OVERLAY = yaml.safe_load((DESCRIPTION / 'config' / 'forklift_mecanum.yaml').read_text())
MECANUM_ROBOT = Model(generate_urdf(
    DESCRIPTION / 'urdf' / 'forklift.urdf.xacro',
    params_overlay=DESCRIPTION / 'config' / 'forklift_mecanum.yaml'))


def robot_launch():
    """Import bringup/launch/robot.launch.py (the launch file has no side effects on import)."""
    path = SRC / 'forklift_bringup' / 'launch' / 'robot.launch.py'
    spec = importlib.util.spec_from_file_location('robot_launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ---------------------------------------------------------------- footprints

@pytest.mark.parametrize('q', [0.0, 0.45])
def test_mecanum_footprint_covers_the_robot(q):
    poly = MECANUM['unloaded']
    for link, pts in MECANUM_ROBOT.collision_points({'fork_lift_joint': q}):
        xy = pts[:, :2]
        assert contains(poly, xy).all(), f'{link} sticks out of the footprint'
        assert edge_distance(poly, xy).min() >= 0.02 - 1e-9, f'{link} too close to the edge'


def test_the_standard_footprint_does_not_cover_the_rear_wheels():
    """This is the reason for a separate footprint."""
    rear = [pts[:, :2] for link, pts in MECANUM_ROBOT.collision_points()
            if link.startswith('rear_') and 'wheel' in link]
    assert len(rear) == 2
    assert not all(contains(STANDARD['unloaded'], xy).all() for xy in rear)


@pytest.mark.parametrize('name', ['unloaded', 'loaded'])
def test_mecanum_footprint_has_the_same_front_section(name):
    """Only the rear section (x < -0.2) is different."""
    front_std = STANDARD[name][STANDARD[name][:, 0] > -0.2]
    front_mec = MECANUM[name][MECANUM[name][:, 0] > -0.2]
    np.testing.assert_allclose(front_std, front_mec)


def test_mecanum_loaded_footprint_contains_unloaded():
    # Grid points inside the unloaded footprint (5 mm or more from its outline).
    xs, ys = np.meshgrid(np.arange(-1.25, 1.42, 0.02), np.arange(-0.61, 0.62, 0.02))
    grid = np.column_stack([xs.ravel(), ys.ravel()])
    inside = grid[contains(MECANUM['unloaded'], grid)
                  & (edge_distance(MECANUM['unloaded'], grid) >= 0.005)]
    assert len(inside) > 1000
    assert contains(MECANUM['loaded'], inside).all()


# ---------------------------------------------------------------- overrides

def nav2_copy():
    import copy
    return copy.deepcopy(NAV2)


def test_every_navigation_override_exists_in_the_nav2_parameters():
    """A new key would be silently ignored by Nav2. A misspelled one is a mistake."""
    for node, values in OVERLAY['navigation'].items():
        params = NAV2[node]['ros__parameters']

        def check(section, reference, path):
            if section.get('plugin', reference.get('plugin')) != reference.get('plugin'):
                return          # another plugin: its parameters are not in the base file
            for key, value in section.items():
                assert key in reference, f'{path}{key} is not in nav2_params.yaml'
                if isinstance(value, dict):
                    check(value, reference[key], f'{path}{key}.')
        check(values, params, f'{node}.')


def test_overrides_are_applied_and_the_rest_is_unchanged():
    launch = robot_launch()
    root = nav2_copy()
    launch.apply_navigation_overrides(root, OVERLAY['navigation'])
    follow = root['controller_server']['ros__parameters']['FollowPath']
    assert follow['desired_linear_vel'] == 1.0
    assert follow['lookahead_dist'] == NAV2['controller_server']['ros__parameters'][
        'FollowPath']['lookahead_dist']
    assert root['velocity_smoother']['ros__parameters']['max_velocity'][0] == 1.0
    assert root['bt_navigator'] == NAV2['bt_navigator']


def test_footprint_override_matches_the_mecanum_footprint_file():
    launch = robot_launch()
    root = nav2_copy()
    launch.apply_navigation_overrides(root, {}, footprint=MECANUM['unloaded'].tolist())
    text = root['local_costmap']['local_costmap']['ros__parameters']['footprint']
    np.testing.assert_allclose(parse_nav2_string(text), MECANUM['unloaded'])


def test_standard_navigation_speed_is_unchanged_by_the_robot_type_mechanism():
    launch = robot_launch()
    root = nav2_copy()
    launch.apply_navigation_overrides(root, None)
    assert root == NAV2


def test_navigation_limits_are_inside_the_hardware_limits():
    controllers = yaml.safe_load(
        (SRC / 'forklift_control' / 'config' / 'controllers_mecanum.yaml').read_text())
    hard = controllers['forklift_base_controller']['ros__parameters']
    nav = OVERLAY['navigation']
    assert nav['controller_server']['FollowPath']['desired_linear_vel'] <= \
        hard['linear.x.max_velocity']
    smoother = nav['velocity_smoother']
    assert smoother['max_velocity'][0] <= hard['linear.x.max_velocity']
    assert smoother['max_velocity'][2] <= hard['angular.z.max_velocity']
    assert smoother['min_velocity'][0] >= hard['linear.x.min_velocity']
    assert smoother['max_accel'][0] <= hard['linear.x.max_acceleration']


def test_unknown_robot_type_is_rejected():
    launch = robot_launch()
    with pytest.raises(RuntimeError, match='robot_type'):
        launch.robot_type_overlay('hovercraft')


def test_every_robot_type_has_its_overlay_file():
    launch = robot_launch()
    assert set(launch.ROBOT_TYPES) == {'standard', 'high_lift', 'mecanum'}
    for name in launch.ROBOT_TYPES:
        path, content = launch.robot_type_overlay(name)
        assert (path == '') == (name == 'standard')
        assert isinstance(content, dict)


def test_mecanum_controller_is_omnidirectional_and_the_smoother_allows_sideways_speed():
    follow = OVERLAY['navigation']['controller_server']['FollowPath']
    assert follow['plugin'].startswith('nav2_mppi_controller')
    assert follow['motion_model'] == 'Omni'
    smoother = OVERLAY['navigation']['velocity_smoother']
    assert follow['vy_max'] > 0
    assert smoother['max_velocity'][1] >= follow['vy_max']
    assert smoother['min_velocity'][1] <= -follow['vy_max']
    assert smoother['max_velocity'][0] >= follow['vx_max']
    controllers = yaml.safe_load((SRC / 'forklift_control' / 'config' /
                                  'controllers_mecanum.yaml').read_text())
    hard = controllers['forklift_base_controller']['ros__parameters']
    assert follow['vy_max'] <= hard['linear.y.max_velocity']


def test_mppi_sampled_accelerations_are_not_the_smoother_limits():
    """Low values (0.8 / 0.5) made the MPPI output rise by 0.01 m/s^2; the smoother limits."""
    follow = OVERLAY['navigation']['controller_server']['FollowPath']
    smoother = OVERLAY['navigation']['velocity_smoother']
    assert follow['ax_max'] >= 2.0 and follow['ay_max'] >= 2.0 and follow['az_max'] >= 2.0
    assert follow['ax_max'] > smoother['max_accel'][0]
    assert -follow['ax_min'] > smoother['max_accel'][0]
