"""Command-state tests for dashboard telemetry simulation."""

from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')

from urc_rover_console.rover_simulator import RoverSimulator  # noqa: E402


class RecordingLogger:
    """Minimal logger accepted by the command callback."""

    def info(self, _message):
        pass

    def warning(self, _message):
        pass


class FakeSimulator:
    """Plain object that exercises RoverSimulator's command logic without ROS."""

    receive_command = RoverSimulator.receive_command

    def __init__(self, state='NAVIGATING', distance=12.0):
        self.navigation_state = state
        self.current_target = 'Active GNSS waypoint'
        self.distance_to_target = distance
        self._logger = RecordingLogger()

    def get_logger(self):
        return self._logger


def command(name):
    """Build the message shape consumed by receive_command."""
    return SimpleNamespace(data=name)


def test_stop_then_start_preserves_remaining_distance():
    simulator = FakeSimulator(distance=12.0)

    simulator.receive_command(command('STOP_MISSION'))
    simulator.receive_command(command('START_MISSION'))

    assert simulator.navigation_state == 'NAVIGATING'
    assert simulator.distance_to_target == 12.0


def test_start_after_terminal_state_begins_a_fresh_attempt():
    simulator = FakeSimulator(state='ABORTED', distance=0.0)

    simulator.receive_command(command('START_MISSION'))

    assert simulator.navigation_state == 'NAVIGATING'
    assert simulator.current_target == 'Active GNSS waypoint'
    assert simulator.distance_to_target == 24.0


@pytest.mark.parametrize(
    'name, expected_state',
    [
        ('ABORT_MISSION', 'ABORTED'),
        ('RESET_MISSION', 'IDLE'),
        ('CANCEL_TARGET', 'IDLE'),
    ],
)
def test_terminal_commands_clear_dashboard_target(name, expected_state):
    simulator = FakeSimulator()

    simulator.receive_command(command(name))

    assert simulator.navigation_state == expected_state
    assert simulator.current_target == '--'
    assert simulator.distance_to_target == 0.0
