"""Focused command-state tests for the coordinate rover simulator."""

from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('geometry_msgs')

from urc_gui_phase2.rover_sim_node import RoverSimNode  # noqa: E402


class RecordingLogger:
    """Minimal logger that records messages emitted by callback methods."""

    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(('info', message))

    def warning(self, message):
        self.messages.append(('warning', message))


def make_simulator():
    """Create callback state without initializing a ROS node or context."""
    simulator = object.__new__(RoverSimNode)
    simulator._east = 7.0
    simulator._north = -4.0
    simulator._target = (20.0, 10.0)
    simulator._motion_enabled = True
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
    assert simulator._target == (20.0, 10.0)
    assert (simulator._east, simulator._north) == (7.0, -4.0)

    simulator._on_command(command('START_MISSION'))
    assert simulator._motion_enabled is True
    assert simulator._target == (20.0, 10.0)
    assert (simulator._east, simulator._north) == (7.0, -4.0)


@pytest.mark.parametrize('name', ['ABORT_MISSION', 'CANCEL_TARGET'])
def test_abort_and_cancel_stop_motion_and_clear_the_target(name):
    simulator = make_simulator()

    simulator._on_command(command(name))

    assert simulator._motion_enabled is False
    assert simulator._target is None
    assert (simulator._east, simulator._north) == (7.0, -4.0)


def test_reset_stops_clears_target_and_returns_to_spawn():
    simulator = make_simulator()

    simulator._on_command(command('RESET_MISSION'))

    assert simulator._motion_enabled is False
    assert simulator._target is None
    assert (simulator._east, simulator._north) == (0.0, 0.0)


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
