"""The controller configuration must agree with the parameters of the robot description."""

import math
import pathlib

import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
CONTROLLERS = yaml.safe_load((PKG / 'config' / 'controllers.yaml').read_text())
MUX = yaml.safe_load((PKG / 'config' / 'twist_mux.yaml').read_text())


def description_params():
    candidates = [PKG.parent / 'forklift_description' / 'config' / 'forklift_params.yaml']
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(pathlib.Path(get_package_share_directory('forklift_description'))
                          / 'config' / 'forklift_params.yaml')
    except Exception:  # noqa: BLE001 - package index not available
        pass
    for c in candidates:
        if c.exists():
            return yaml.safe_load(c.read_text())
    pytest.skip('forklift_description params not found')


def test_wheel_geometry_matches_description():
    p = description_params()
    base = CONTROLLERS['forklift_base_controller']['ros__parameters']
    assert base['wheel_radius'] == pytest.approx(p['drive_wheel']['radius'])
    assert base['wheel_separation'] == pytest.approx(p['drive_wheel']['separation'])


def test_velocity_envelope_within_wheel_limits():
    p = description_params()
    base = CONTROLLERS['forklift_base_controller']['ros__parameters']
    wheel_speed_limit = p['drive_wheel']['max_velocity'] * p['drive_wheel']['radius']
    half_track = p['drive_wheel']['separation'] / 2
    assert base['linear.x.max_velocity'] <= wheel_speed_limit
    # Turn in place: the wheels must be able to get the rim speed = omega * half track.
    assert base['angular.z.max_velocity'] * half_track <= wheel_speed_limit


def test_braking_matches_tip_over_assumption():
    # The tip-over test of forklift_description uses a braking deceleration of 0.5 m/s^2 or less.
    base = CONTROLLERS['forklift_base_controller']['ros__parameters']
    assert abs(base['linear.x.max_deceleration']) <= 0.5
    assert abs(base['linear.x.min_acceleration']) <= 0.5


def test_odometry_does_not_publish_tf():
    base = CONTROLLERS['forklift_base_controller']['ros__parameters']
    assert base['enable_odom_tf'] is False
    assert base['base_frame_id'] == 'base_footprint'
    assert base['odom_frame_id'] == 'odom'


def test_estop_has_highest_priority_and_no_timeout():
    mux = MUX['twist_mux']['ros__parameters']
    estop = mux['locks']['estop']
    assert estop['timeout'] == 0.0
    assert all(estop['priority'] > t['priority'] for t in mux['topics'].values())
    assert mux['use_stamped'] is True


def test_load_handler_geometry_matches_description():
    from forklift_description.urdf_kinematics import generate_urdf, Model
    import numpy as np
    handler = yaml.safe_load((PKG / 'config' / 'load_handler.yaml').read_text())
    hp = handler['load_handler']['ros__parameters']
    p = description_params()
    model = Model(generate_urdf())
    assert hp['fork_min_height'] == pytest.approx(p['lift']['min_height'])
    assert hp['fork_thickness'] == pytest.approx(p['forks']['thickness'])
    assert hp['heel_x'] == pytest.approx(model.point('fork_carriage_link', np.zeros(3))[0])
    assert hp['lidar_x'] == pytest.approx(model.point('lidar_link', np.zeros(3))[0])


# ----------------------------------------------------------------------------- Mecanum robot

MECANUM = yaml.safe_load((PKG / 'config' / 'controllers_mecanum.yaml').read_text())
WHEEL_JOINTS = ['front_left_drive_wheel_joint', 'front_right_drive_wheel_joint',
                'rear_left_drive_wheel_joint', 'rear_right_drive_wheel_joint']


def mecanum_params():
    candidates = [PKG.parent / 'forklift_description' / 'config' / 'forklift_mecanum.yaml']
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(pathlib.Path(get_package_share_directory('forklift_description'))
                          / 'config' / 'forklift_mecanum.yaml')
    except Exception:  # noqa: BLE001 - package index not available
        pass
    for c in candidates:
        if c.exists():
            return yaml.safe_load(c.read_text())
    pytest.skip('forklift_mecanum.yaml not found')


def test_mecanum_controller_type_and_name_are_compatible():
    manager = MECANUM['controller_manager']['ros__parameters']
    # The same name as the differential controller: launch files and tests wait for it.
    assert manager['forklift_base_controller']['type'] == \
        'mecanum_drive_controller/MecanumDriveController'
    assert set(manager) == set(CONTROLLERS['controller_manager']['ros__parameters'])


def test_mecanum_wheel_joint_names_match_the_description():
    base = MECANUM['forklift_base_controller']['ros__parameters']
    for corner in ('front_left', 'front_right', 'rear_left', 'rear_right'):
        joint = f'{corner}_drive_wheel_joint'
        assert joint in WHEEL_JOINTS
        assert base[f'{corner}_wheel_command_joint_name'] == joint
        assert base[f'{corner}_wheel_state_joint_name'] == joint


def test_mecanum_geometry_matches_description():
    p = mecanum_params()['drive_wheel']
    kin = MECANUM['forklift_base_controller']['ros__parameters']['kinematics']
    assert kin['wheels_radius'] == pytest.approx(p['radius'])
    # lx + ly: the distance of a wheel to the center of the wheel rectangle (projected).
    assert kin['sum_of_robot_center_projection_on_X_Y_axis'] == pytest.approx(
        (p['wheelbase'] + p['separation']) / 2)
    # base_footprint is above the front axle: the rectangle center is wheelbase / 2 behind it.
    assert kin['base_frame_offset']['x'] == pytest.approx(p['wheelbase'] / 2)
    assert kin['base_frame_offset']['y'] == pytest.approx(0.0)
    assert kin['base_frame_offset']['theta'] == pytest.approx(0.0)


def test_mecanum_velocity_envelope_within_wheel_limits():
    p = mecanum_params()['drive_wheel']
    base = MECANUM['forklift_base_controller']['ros__parameters']
    rim_speed = p['max_velocity'] * p['radius']
    reach = (p['wheelbase'] + p['separation']) / 2
    assert base['linear.x.max_velocity'] <= rim_speed
    assert base['linear.y.max_velocity'] <= rim_speed
    assert -base['linear.x.min_velocity'] <= rim_speed
    assert -base['linear.y.min_velocity'] <= rim_speed
    assert base['angular.z.max_velocity'] * reach <= rim_speed


def test_mecanum_odometry_does_not_publish_tf():
    base = MECANUM['forklift_base_controller']['ros__parameters']
    assert base['enable_odom_tf'] is False
    assert base['base_frame_id'] == 'base_footprint'
    assert base['odom_frame_id'] == 'odom'


def test_mecanum_lift_controller_equals_the_differential_one():
    assert MECANUM['fork_lift_controller'] == CONTROLLERS['fork_lift_controller']


def test_robot_type_load_handler_overrides_are_known_parameters_and_faster_for_mecanum():
    """An overlay key that the node does not declare would silently do nothing."""
    base = yaml.safe_load((PKG / 'config' / 'load_handler.yaml').read_text())
    base = base['load_handler']['ros__parameters']
    overlay = yaml.safe_load((PKG.parent / 'forklift_description' / 'config' /
                              'forklift_mecanum.yaml').read_text())['load_handler']
    assert set(overlay) <= set(base)
    for key in ('approach_speed', 'insert_speed', 'reverse_speed'):
        assert overlay[key] > base[key], key
    source = (PKG / 'forklift_control' / 'load_handler.py').read_text()
    for key in base:
        if key.endswith('_speed') or key == 'dock_decel':
            assert f"'{key}'" in source, f'{key} is not read by load_handler.py'


def test_load_handler_declares_each_parameter_only_once():
    """A duplicate declaration raises ParameterAlreadyDeclaredException at start-up."""
    import collections
    import re
    source = (PKG / 'forklift_control' / 'load_handler.py').read_text()
    names = re.findall(r"\bp\('([a-z_0-9]+)'", source)
    duplicates = [n for n, c in collections.Counter(names).items() if c > 1]
    assert not duplicates, duplicates


def test_nav2_goal_tolerance_is_inside_the_handler_staging_tolerance():
    """A staging pose that Nav2 accepts must be accepted by the skills (no second Nav2 run)."""
    nav = PKG.parent / 'forklift_navigation' / 'config' / 'nav2_params.yaml'
    if not nav.exists():
        pytest.skip('navigation package is not in the source tree')
    goal = yaml.safe_load(nav.read_text())['controller_server']['ros__parameters'][
        'goal_checker']
    handler = yaml.safe_load((PKG / 'config' / 'load_handler.yaml').read_text())
    hp = handler['load_handler']['ros__parameters']
    assert goal['xy_goal_tolerance'] <= hp['staging_tolerance']
    assert math.degrees(goal['yaw_goal_tolerance']) <= hp['staging_yaw_tolerance_deg']


def test_mecanum_overlay_enables_sideways_alignment_and_the_node_reads_it():
    overlay = yaml.safe_load((PKG.parent / 'forklift_description' / 'config' /
                              'forklift_mecanum.yaml').read_text())['load_handler']
    assert overlay['holonomic'] is True
    base = yaml.safe_load((PKG / 'config' / 'load_handler.yaml').read_text())
    assert base['load_handler']['ros__parameters']['holonomic'] is False
    source = (PKG / 'forklift_control' / 'load_handler.py').read_text()
    for key in ('holonomic', 'insert_lateral_gain', 'strafe_speed', 'approach_strafe_speed',
                'insert_strafe_speed',
                'stationary_time'):
        assert f"'{key}'" in source, key
