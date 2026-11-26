"""fleet.yaml: the capabilities of the robots agree with the robot types."""

import pathlib

from forklift_fleet.allocator import RobotView, unassignable
import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
DESCRIPTION = PKG.parent / 'forklift_description' / 'config'
FLEET = yaml.safe_load((PKG / 'config' / 'fleet.yaml').read_text())['fleet_coordinator'][
    'ros__parameters']
OVERLAYS = {'high_lift': 'forklift_high_lift.yaml', 'mecanum': 'forklift_mecanum.yaml'}


@pytest.mark.parametrize('robot', FLEET['robots'])
def test_capabilities_agree_with_the_robot_type(robot):
    kind = FLEET[f'robot_type.{robot}']
    path = DESCRIPTION / OVERLAYS.get(kind, 'forklift_params.yaml')
    if not path.exists():
        pytest.skip('description package is not in the source tree')
    lift = yaml.safe_load(path.read_text())['lift']
    assert FLEET[f'max_load_kg.{robot}'] == pytest.approx(lift['rated_load'])
    assert FLEET[f'max_lift_height.{robot}'] == pytest.approx(lift['max_travel'])


def test_every_robot_has_speed_and_handling_time():
    for robot in FLEET['robots']:
        assert FLEET[f'speed.{robot}'] > 0 and FLEET[f'handling_time.{robot}'] > 0


def test_the_mecanum_robot_is_faster_and_handles_quicker_than_the_differential_one():
    kinds = {FLEET[f'robot_type.{r}']: r for r in FLEET['robots']}
    fast, slow = kinds['mecanum'], kinds['high_lift']
    assert FLEET[f'speed.{fast}'] > FLEET[f'speed.{slow}']
    assert FLEET[f'handling_time.{fast}'] < FLEET[f'handling_time.{slow}']


def test_the_default_mission_is_possible_with_this_fleet():
    from forklift_task_manager.mission_config import default_mission_path, load_mission
    mission = load_mission(default_mission_path())
    robots = [RobotView(r, 0.0, 0.0, max_load_kg=FLEET[f'max_load_kg.{r}'],
                        max_lift=FLEET[f'max_lift_height.{r}']) for r in FLEET['robots']]
    from forklift_fleet.allocator import TaskView
    tasks = [TaskView(b.box_id, (0, 0), (0, 0), mass_kg=mission.load.mass,
                      lift_height=mission.destinations[b.destination].surface_height +
                      FLEET['lift_margin']) for b in mission.boxes]
    assert unassignable(robots, tasks) == []


def test_the_lift_margin_agrees_with_the_placement_height_of_the_handler():
    """The coordinator estimates the fork height of a placement as surface + lift_margin."""
    from forklift_control.load_handling_logic import fork_heights
    from forklift_task_manager.mission_config import default_mission_path, load_mission
    mission = load_mission(default_mission_path())
    handler = yaml.safe_load((PKG.parent / 'forklift_control' / 'config' /
                              'load_handler.yaml').read_text())['load_handler']['ros__parameters']
    surface = 0.25
    h = fork_heights(mission.load, surface, handler['fork_min_height'],
                     handler['fork_thickness'], handler['insert_fork_top'],
                     handler['transport_clearance'], handler['place_clearance'],
                     handler['release_gap'])
    assert h.place_raised == pytest.approx(surface + FLEET['lift_margin'], abs=0.01)
