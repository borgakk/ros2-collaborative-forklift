"""Logic of the lifecycle starter that does not need ROS: the watchdog of lost responses."""

from forklift_navigation.lifecycle_starter import ReplyWatchdog


def test_new_clients_are_requested_after_three_lost_responses_in_a_row():
    w = ReplyWatchdog(limit=3)
    assert [w.lost(), w.lost(), w.lost()] == [False, False, True]
    assert [w.lost(), w.lost(), w.lost()] == [False, False, True]     # counts again


def test_an_answer_resets_the_count():
    w = ReplyWatchdog(limit=3)
    w.lost()
    w.lost()
    w.answered()
    assert [w.lost(), w.lost(), w.lost()] == [False, False, True]


def test_bt_navigator_waits_long_enough_for_the_action_servers():
    import pathlib
    import yaml
    params = yaml.safe_load((pathlib.Path(__file__).resolve().parents[1] / 'config' /
                             'nav2_params.yaml').read_text())
    assert params['bt_navigator']['ros__parameters']['wait_for_service_timeout'] >= 5000
