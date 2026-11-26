"""Readiness of the Nav2 stack for the dispatch of tasks."""

from forklift_fleet.fleet_agent import NAV_NODES, nav_stack_ready, systems_alive


def test_ready_only_if_every_nav2_node_is_active():
    assert nav_stack_ready({n: True for n in NAV_NODES})
    states = {n: True for n in NAV_NODES}
    states['bt_navigator'] = False
    assert not nav_stack_ready(states)


def test_unknown_state_is_not_ready():
    """A lost service response leaves the state unknown. Unknown is not ready."""
    assert not nav_stack_ready({})
    assert not nav_stack_ready({n: True for n in NAV_NODES[:-1]})


def test_nav2_node_list_agrees_with_the_launch_file():
    import pathlib
    import re
    launch = (pathlib.Path(__file__).resolve().parents[2] / 'forklift_navigation' / 'launch' /
              'navigation.launch.py')
    if not launch.exists():
        import pytest
        pytest.skip('navigation package is not in the source tree')
    text = launch.read_text()
    block = text[text.index('NAV2_NODES'):text.index(']', text.index('NAV2_NODES')) + 1]
    assert re.findall(r"'([a-z_]+)'", block) == NAV_NODES


def test_systems_are_alive_only_if_odometry_and_fork_state_are_recent():
    assert systems_alive(100.0, 99.0, 98.5)
    assert not systems_alive(100.0, 99.0, 90.0)        # no fork state: no lift controller
    assert not systems_alive(100.0, 90.0, 99.0)
    assert not systems_alive(100.0, -1e9, -1e9)        # nothing received yet
