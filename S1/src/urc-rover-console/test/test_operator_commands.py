"""Focused tests for command safety in the combined rover console."""

import json
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
rclpy_executor_stub = ModuleType('rclpy.executors')
rclpy_bindings_stub = ModuleType('rclpy._rclpy_pybind11')


class _UnusedSingleThreadedExecutor:
    pass


class _RCLError(Exception):
    pass


rclpy_executor_stub.SingleThreadedExecutor = _UnusedSingleThreadedExecutor
rclpy_bindings_stub.RCLError = _RCLError
std_msgs_stub = ModuleType('std_msgs')
std_msgs_msg_stub = ModuleType('std_msgs.msg')
std_msgs_msg_stub.String = _String

stubs = {
    'rclpy': rclpy_stub,
    'rclpy.executors': rclpy_executor_stub,
    'rclpy._rclpy_pybind11': rclpy_bindings_stub,
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
    from urc_rover_console.battery_gui import RoverConsole, spin_ready_callbacks
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


class FakeMonitor:
    """Controllable replacement for the monotonic-time stream monitor."""

    def __init__(self, state='active', age=0.0):
        self.state = state
        self.age = age

    def mark_received(self):
        self.state = 'active'
        self.age = 0.0

    def connection_state(self):
        age = None if self.state == 'waiting' else self.age
        return self.state, age


class FakeEventLog:
    """Capture structured events separately from published ROS messages."""

    def __init__(self):
        self.events = []

    def record(self, category, event, **details):
        self.events.append((category, event, details))


class FakeMissionPanel:
    def __init__(self):
        self.reports = []

    def report(self, text, ok):
        self.reports.append((text, ok))


class FakeMissionController:
    def __init__(self, active_target, frame_available, next_waypoint=None):
        self._active_target = active_target
        self._next_waypoint = next_waypoint
        self.completed_count = 0
        self.frame = (
            SimpleNamespace(origin_lat_deg=38.0, origin_lon_deg=-110.0)
            if frame_available else None
        )

    def active_target(self):
        return self._active_target

    def active_target_enu(self):
        return SimpleNamespace(east_m=12.5, north_m=-3.0)

    def complete_active(self):
        if self._active_target is None:
            raise ValueError('no active waypoint to complete')
        self.completed_count += 1
        self._active_target = self._next_waypoint
        return self._next_waypoint


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

    def set_obstacle_enu(self, **obstacle):
        self._events.append(('map', 'obstacle', obstacle))

    def set_obstacles_enu(self, obstacles):
        self._events.append(('map', 'obstacles', tuple(obstacles)))

    def set_planned_route_enu(self, route):
        self._events.append(('map', 'route', tuple(route)))


class FakeConsole:
    send_command = RoverConsole.send_command
    publish_active_target = RoverConsole.publish_active_target
    update_mission_status = RoverConsole.update_mission_status
    complete_arrived_waypoint = RoverConsole.complete_arrived_waypoint
    data_stream_states = RoverConsole.data_stream_states
    critical_data_failures = RoverConsole.critical_data_failures
    send_safety_stop_once = RoverConsole.send_safety_stop_once

    def __init__(self, active_target, frame_available=True, next_waypoint=None):
        self.events = []
        self.mission_controller = FakeMissionController(
            active_target,
            frame_available,
            next_waypoint,
        )
        self.feedback = FakeLabel()
        self.target = FakeLabel()
        self.nav_state = FakeLabel()
        self.distance = FakeLabel()
        self.banner = FakeLabel()
        self.mission_panel = FakeMissionPanel()
        self.command_publisher = FakePublisher(self.events)
        self.node = FakeNode(self.events)
        self.rover_track = FakeTrack(self.events)
        self.map_widget = FakeMapWidget(self.events)
        self.event_log = FakeEventLog()
        self.fix_monitor = FakeMonitor()
        self.status_monitor = FakeMonitor()
        self.telemetry_monitor = FakeMonitor()
        self._last_mission_state = None
        self._current_mission_state = 'IDLE'
        self._completing_arrival = False
        self._safety_stop_sent = False


def test_ros_callback_processing_is_bounded_per_qt_timer_tick():
    class FakeExecutor:
        def __init__(self):
            self.timeouts = []

        def spin_once(self, timeout_sec):
            self.timeouts.append(timeout_sec)

    executor = FakeExecutor()

    spin_ready_callbacks(executor, max_callbacks=3)

    assert executor.timeouts == [0, 0, 0]


def test_start_without_active_waypoint_is_blocked():
    console = FakeConsole(active_target=None)

    assert console.send_command('START_MISSION') is False
    assert console.events == []
    assert 'set an active waypoint' in console.feedback.text
    assert console.mission_panel.reports[-1][1] is False


def test_start_is_blocked_when_critical_ros_data_is_missing():
    console = FakeConsole(active_target=object())
    console.fix_monitor.state = 'waiting'

    assert console.send_command('START_MISSION') is False
    assert console.events == []
    assert 'fresh gnss data' in console.feedback.text


def test_start_publishes_target_before_movement_command():
    waypoint = SimpleNamespace(
        id='wp-1',
        name='Sample Site',
        lat_deg=38.1,
        lon_deg=-110.1,
    )
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


def mission_status(state, has_target, distance_m, **extra):
    """Build the JSON message published by the coordinate rover."""
    payload = {
        'state': state,
        'has_target': has_target,
        'distance_m': distance_m,
    }
    payload.update(extra)
    return SimpleNamespace(
        data=json.dumps(payload)
    )


def test_authoritative_status_updates_navigation_labels():
    active = SimpleNamespace(id='wp-1', name='Sample Site')
    console = FakeConsole(active_target=active)

    console.update_mission_status(
        mission_status('NAVIGATING', True, 8.42)
    )

    assert console.target.text == 'Target: Sample Site'
    assert console.nav_state.text == 'State: NAVIGATING'
    assert console.distance.text == 'Distance: 8.4 m'
    assert console.banner.text == 'NAVIGATING — Sample Site'


def test_arrival_completes_waypoint_only_once_for_repeated_status():
    active = SimpleNamespace(id='wp-1', name='Sample Site')
    console = FakeConsole(active_target=active)
    arrived = mission_status('ARRIVED', True, 0.75)

    console.update_mission_status(arrived)
    console.update_mission_status(arrived)

    assert console.mission_controller.completed_count == 1
    assert console.feedback.text == 'Reached Sample Site; mission complete.'


def test_arrival_prepares_next_waypoint_without_starting_it():
    active = SimpleNamespace(id='wp-1', name='First Site')
    next_waypoint = SimpleNamespace(id='wp-2', name='Second Site')
    console = FakeConsole(
        active_target=active,
        next_waypoint=next_waypoint,
    )

    console.update_mission_status(
        mission_status('ARRIVED', True, 0.5)
    )

    assert console.mission_controller.active_target() is next_waypoint
    assert 'Second Site is ready' in console.feedback.text
    assert 'Press START to continue' in console.feedback.text


def test_invalid_mission_status_does_not_change_navigation_labels():
    console = FakeConsole(active_target=None)

    console.update_mission_status(
        mission_status('FLYING', False, -1.0)
    )

    assert console.nav_state.text == ''
    assert console.feedback.text == 'INVALID MISSION STATUS MESSAGE'


def test_avoiding_status_displays_obstacle_route():
    active = SimpleNamespace(id='wp-1', name='East Site')
    console = FakeConsole(active_target=active)

    console.update_mission_status(
        mission_status(
            'AVOIDING',
            True,
            24.0,
            avoidance_planned=True,
            route=[
                {'east_m': 15.0, 'north_m': 6.0},
                {'east_m': 30.0, 'north_m': 0.0},
            ],
            obstacle={
                'east_m': 15.0,
                'north_m': 0.0,
                'radius_m': 3.0,
                'clearance_m': 2.0,
            },
            planning_error=None,
        )
    )

    assert console.banner.text == 'AVOIDING OBSTACLE — East Site'
    assert console.events[-2][0:2] == ('map', 'obstacles')
    assert console.events[-1][0:2] == ('map', 'route')


def test_unreachable_status_records_failed_target_result():
    active = SimpleNamespace(id='wp-1', name='Blocked Site')
    console = FakeConsole(active_target=active)

    console.update_mission_status(
        mission_status(
            'UNREACHABLE',
            False,
            None,
            planning_error='goal lies inside the safety boundary',
        )
    )

    assert console.banner.text == 'TARGET UNREACHABLE — Blocked Site'
    assert 'goal lies inside' in console.feedback.text
    assert console.event_log.events[-1][0:2] == (
        'target_result',
        'unreachable',
    )


def test_safety_stop_is_sent_only_once_for_one_outage():
    console = FakeConsole(active_target=object())

    console.send_safety_stop_once(['gnss'])
    console.send_safety_stop_once(['gnss'])

    assert console.events == [('command', 'STOP_MISSION')]
    assert 'fresh gnss data became unavailable' in console.feedback.text
