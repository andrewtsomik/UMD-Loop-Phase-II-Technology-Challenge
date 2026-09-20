"""Unit tests for missing and stale ROS stream classification."""

import pytest

from urc_rover_console.telemetry_monitor import TelemetryMonitor


class FakeClock:
    """Small manually advanced monotonic clock."""

    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def test_stream_is_waiting_before_its_first_message():
    monitor = TelemetryMonitor(clock=FakeClock())

    assert monitor.connection_state() == ("waiting", None)


def test_recent_message_is_active_then_becomes_stale():
    clock = FakeClock()
    monitor = TelemetryMonitor(stale_after_seconds=2.5, clock=clock)
    monitor.mark_received()

    clock.now += 2.5
    assert monitor.connection_state() == ("active", 2.5)

    clock.now += 0.01
    state, age = monitor.connection_state()
    assert state == "stale"
    assert age == pytest.approx(2.51)


def test_new_message_recovers_a_stale_stream():
    clock = FakeClock()
    monitor = TelemetryMonitor(stale_after_seconds=1.0, clock=clock)
    monitor.mark_received()
    clock.now += 2.0
    assert monitor.connection_state()[0] == "stale"

    monitor.mark_received()

    assert monitor.connection_state() == ("active", 0.0)


@pytest.mark.parametrize("timeout", [0.0, -1.0, float("nan"), float("inf")])
def test_timeout_must_be_positive(timeout):
    with pytest.raises(ValueError):
        TelemetryMonitor(stale_after_seconds=timeout)
