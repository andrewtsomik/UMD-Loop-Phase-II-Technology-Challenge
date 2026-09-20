"""Focused command-state tests for the coordinate rover simulator."""

import json
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('geometry_msgs')

from builtin_interfaces.msg import Time  # noqa: E402
from urc_gui_phase2.obstacle_planner import CircleObstacle  # noqa: E402
from urc_gui_phase2.rover_sim_node import RoverSimNode  # noqa: E402


class RecordingLogger:
    """Minimal logger that records messages emitted by callback methods."""

    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(('info', message))

    def warning(self, message):
        self.messages.append(('warning', message))


class RecordingPublisher:
    """Capture the most recently published ROS message."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_simulator():
    """Create callback state without initializing a ROS node or context."""
    simulator = object.__new__(RoverSimNode)
    simulator._east = 7.0
    simulator._north = -4.0
    simulator._target = (20.0, 10.0)
    simulator._route = [(20.0, 10.0)]
    simulator._avoidance_planned = False
    simulator._planning_error = None
    simulator._obstacle = None
    simulator._obstacle_clearance = 1.0
    simulator._motion_enabled = True
    simulator._state = 'NAVIGATING'
    logger = RecordingLogger()
    simulator.get_logger = lambda: logger
    return simulator


def command(name):
    """Build the small message shape consumed by the command callback."""
    return SimpleNamespace(data=name)


def test_stop_then_start_resumes_with_the_same_target_and_position():
    simulator = make_simulator()

    simulator._on_command(command('STOP_MISSION'))
    assert simulator._motion_enabled is False
    assert simulator._state == 'STOPPED'
    assert simulator._target == (20.0, 10.0)
    assert simulator._route == [(20.0, 10.0)]
    assert (simulator._east, simulator._north) == (7.0, -4.0)

    simulator._on_command(command('START_MISSION'))
    assert simulator._motion_enabled is True
    assert simulator._state == 'NAVIGATING'
    assert simulator._target == (20.0, 10.0)
    assert simulator._route == [(20.0, 10.0)]
    assert (simulator._east, simulator._north) == (7.0, -4.0)


@pytest.mark.parametrize(
    ('name', 'expected_state'),
    [('ABORT_MISSION', 'ABORTED'), ('CANCEL_TARGET', 'IDLE')],
)
def test_abort_and_cancel_stop_motion_and_clear_the_target(
        name, expected_state):
    simulator = make_simulator()

    simulator._on_command(command(name))

    assert simulator._motion_enabled is False
    assert simulator._target is None
    assert simulator._route == []
    assert simulator._state == expected_state
    assert (simulator._east, simulator._north) == (7.0, -4.0)


def test_reset_stops_clears_target_and_returns_to_spawn():
    simulator = make_simulator()

    simulator._on_command(command('RESET_MISSION'))

    assert simulator._motion_enabled is False
    assert simulator._target is None
    assert simulator._route == []
    assert simulator._state == 'IDLE'
    assert (simulator._east, simulator._north) == (0.0, 0.0)


def test_start_without_a_target_is_rejected():
    simulator = make_simulator()
    simulator._target = None
    simulator._route = []

    simulator._on_command(command('START_MISSION'))

    assert simulator._motion_enabled is False
    assert simulator._state == 'IDLE'


def test_target_validation_preserves_last_valid_target():
    simulator = make_simulator()

    wrong_frame = SimpleNamespace(
        header=SimpleNamespace(frame_id='odom'),
        pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0)),
    )
    simulator._on_target(wrong_frame)
    assert simulator._target == (20.0, 10.0)

    invalid = SimpleNamespace(
        header=SimpleNamespace(frame_id='map'),
        pose=SimpleNamespace(position=SimpleNamespace(x=float('nan'), y=2.0)),
    )
    simulator._on_target(invalid)
    assert simulator._target == (20.0, 10.0)

    valid = SimpleNamespace(
        header=SimpleNamespace(frame_id='map'),
        pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0)),
    )
    simulator._on_target(valid)
    assert simulator._target == (1.0, 2.0)
    assert simulator._route == [(1.0, 2.0)]


def test_tick_reports_real_arrival_and_remaining_distance():
    simulator = make_simulator()
    simulator._target = (7.5, -4.0)
    simulator._route = [(7.5, -4.0)]
    simulator._pub = RecordingPublisher()
    simulator._status_pub = RecordingPublisher()
    simulator._frame = SimpleNamespace(
        to_wgs84=lambda east, north: SimpleNamespace(
            lat_deg=38.0,
            lon_deg=-110.0,
            alt_m=0.0,
        )
    )
    simulator.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=Time)
    )

    simulator._tick()

    status = json.loads(simulator._status_pub.messages[-1].data)
    assert simulator._state == 'ARRIVED'
    assert simulator._motion_enabled is False
    assert status['state'] == 'ARRIVED'
    assert status['has_target'] is True
    assert status['distance_m'] == 0.5
    assert status['route'] == []


def test_blocked_target_gets_a_safe_detour_route():
    simulator = make_simulator()
    simulator._east = 0.0
    simulator._north = 0.0
    simulator._motion_enabled = False
    simulator._obstacle = CircleObstacle(10.0, 0.0, 2.0)
    simulator._obstacle_clearance = 1.0
    target = SimpleNamespace(
        header=SimpleNamespace(frame_id='map'),
        pose=SimpleNamespace(position=SimpleNamespace(x=20.0, y=0.0)),
    )

    simulator._on_target(target)

    assert simulator._target == (20.0, 0.0)
    assert len(simulator._route) == 2
    assert simulator._avoidance_planned is True
    assert simulator._state == 'READY'


def test_target_inside_obstacle_is_reported_as_unreachable():
    simulator = make_simulator()
    simulator._east = 0.0
    simulator._north = 0.0
    simulator._motion_enabled = False
    simulator._obstacle = CircleObstacle(10.0, 0.0, 2.0)
    simulator._obstacle_clearance = 1.0
    target = SimpleNamespace(
        header=SimpleNamespace(frame_id='map'),
        pose=SimpleNamespace(position=SimpleNamespace(x=10.0, y=0.0)),
    )

    simulator._on_target(target)

    assert simulator._target is None
    assert simulator._route == []
    assert simulator._state == 'UNREACHABLE'
    assert 'goal lies inside' in simulator._planning_error
