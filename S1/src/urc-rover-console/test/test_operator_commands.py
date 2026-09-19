"""Focused tests for command safety in the combined rover console."""

import sys
from types import ModuleType, SimpleNamespace

import pytest

pytest.importorskip('PyQt5')


class _UnusedMapWidget:
    """Allow importing the console without the optional map browser package."""


class _String:
    def __init__(self):
        self.data = ''


class _UnusedOperatorGuiNode:
    pass


map_widget_stub = ModuleType('urc_gui_phase2.map_widget')
map_widget_stub.OfflineMapWidget = _UnusedMapWidget
operator_node_stub = ModuleType('urc_gui_phase2.operator_gui_node')
operator_node_stub.OperatorGuiNode = _UnusedOperatorGuiNode
rclpy_stub = ModuleType('rclpy')
std_msgs_stub = ModuleType('std_msgs')
std_msgs_msg_stub = ModuleType('std_msgs.msg')
std_msgs_msg_stub.String = _String

stubs = {
    'rclpy': rclpy_stub,
    'std_msgs': std_msgs_stub,
    'std_msgs.msg': std_msgs_msg_stub,
    'urc_gui_phase2.map_widget': map_widget_stub,
    'urc_gui_phase2.operator_gui_node': operator_node_stub,
}
original_modules = {
    name: sys.modules.get(name)
    for name in stubs
}
sys.modules.update(stubs)

try:
    from urc_rover_console.battery_gui import RoverConsole
finally:
    for name, original in original_modules.items():
        if original is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = original


class FakeLabel:
    def __init__(self):
        self.text = ''

    def setText(self, text):
        self.text = text


class FakeMissionPanel:
    def __init__(self):
        self.reports = []

    def report(self, text, ok):
        self.reports.append((text, ok))


class FakeMissionController:
    def __init__(self, active_target, frame_available):
        self._active_target = active_target
        self.frame = object() if frame_available else None

    def active_target(self):
        return self._active_target

    def active_target_enu(self):
        return SimpleNamespace(east_m=12.5, north_m=-3.0)


class FakePublisher:
    def __init__(self, events):
        self._events = events

    def publish(self, message):
        self._events.append(('command', message.data))


class FakeNode:
    def __init__(self, events):
        self._events = events

    def publish_active_target(self, east_m, north_m):
        self._events.append(('target', east_m, north_m))


class FakeTrack:
    def __init__(self, events):
        self._events = events

    def reset(self):
        self._events.append(('track', 'reset'))


class FakeMapWidget:
    def __init__(self, events):
        self._events = events

    def clear_rover_track(self):
        self._events.append(('map', 'clear_rover_track'))


class FakeConsole:
    send_command = RoverConsole.send_command
    publish_active_target = RoverConsole.publish_active_target

    def __init__(self, active_target, frame_available=True):
        self.events = []
        self.mission_controller = FakeMissionController(
            active_target,
            frame_available,
        )
        self.feedback = FakeLabel()
        self.mission_panel = FakeMissionPanel()
        self.command_publisher = FakePublisher(self.events)
        self.node = FakeNode(self.events)
        self.rover_track = FakeTrack(self.events)
        self.map_widget = FakeMapWidget(self.events)


def test_start_without_active_waypoint_is_blocked():
    console = FakeConsole(active_target=None)

    assert console.send_command('START_MISSION') is False
    assert console.events == []
    assert 'set an active waypoint' in console.feedback.text
    assert console.mission_panel.reports[-1][1] is False


def test_start_publishes_target_before_movement_command():
    waypoint = object()
    console = FakeConsole(active_target=waypoint)

    assert console.send_command('START_MISSION') is True
    assert console.events == [
        ('target', 12.5, -3.0),
        ('command', 'START_MISSION'),
    ]


def test_start_is_blocked_when_target_cannot_be_published():
    console = FakeConsole(
        active_target=object(),
        frame_available=False,
    )

    assert console.send_command('START_MISSION') is False
    assert console.events == []
    assert 'could not be sent' in console.feedback.text
    assert 'local coordinate frame' in console.mission_panel.reports[-1][0]


def test_non_start_command_does_not_require_active_waypoint():
    console = FakeConsole(active_target=None)

    assert console.send_command('STOP_MISSION') is True
    assert console.events == [('command', 'STOP_MISSION')]


def test_reset_clears_track_after_command_is_published():
    console = FakeConsole(active_target=None)

    assert console.send_command('RESET_MISSION') is True
    assert console.events == [
        ('command', 'RESET_MISSION'),
        ('track', 'reset'),
        ('map', 'clear_rover_track'),
    ]


def test_removing_active_waypoint_sends_target_cancellation():
    console = FakeConsole(active_target=None)

    assert console.publish_active_target(None) is False
    assert console.events == [('command', 'CANCEL_TARGET')]
