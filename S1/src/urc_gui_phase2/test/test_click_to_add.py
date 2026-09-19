"""Click-to-add through the whole GUI shell: real map -> OperatorWindow -> controller -> panel.

Adding needs the panel's one-shot "add waypoint on click" mode; selecting never does.
"""
import json
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('QTWEBENGINE_CHROMIUM_FLAGS', '--no-sandbox --disable-gpu')

import pytest

pytest.importorskip('PyQt5.QtWebEngineWidgets')
pytest.importorskip('pyqtlet2')
pytest.importorskip('rclpy')
pytest.importorskip('geometry_msgs')

from PyQt5.QtCore import QCoreApplication, Qt  # noqa: E402
from PyQt5.QtTest import QTest  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

# Must precede the first QApplication so QtWebEngine can share the process.
QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
_app = QApplication.instance() or QApplication([])

from urc_gui_phase2 import operator_gui_node as gui  # noqa: E402
from urc_gui_phase2.coordinate_convert import LocalFrame  # noqa: E402
from urc_gui_phase2.map_geometry import bbox_around, meters_per_pixel, tiles_for_bbox  # noqa: E402
from urc_gui_phase2.mission_controller import MissionController  # noqa: E402
from urc_gui_phase2.mission_model import TargetType  # noqa: E402

MDRS = (38.4058, -110.7919)
PNG = bytes.fromhex('89504e470d0a1a0a0000000d49484452000000010000000108060000001f15c489'
                    '0000000d49444154789c6360f8cfc0f00f0001040100e0c4fc4d0000000049454e44ae426082')


def wait_for(predicate, timeout=10.0):
    import time
    end = time.time() + timeout
    while time.time() < end:
        if predicate():
            return True
        QTest.qWait(50)
    return predicate()


class FakeLogger:
    def error(self, *_):
        pass


class FakeNode:
    """The bits of OperatorGuiNode that OperatorWindow touches (no rclpy context needed)."""
    on_fix = None

    def __init__(self):
        self.published = []

    def publish_active_target(self, east_m, north_m):
        self.published.append((east_m, north_m))

    def get_logger(self):
        return FakeLogger()


@pytest.fixture(scope='module')
def tile_dir(tmp_path_factory):
    root = tmp_path_factory.mktemp('tiles')
    box = bbox_around(*MDRS, 2500)
    for z in range(12, 18):
        for x, y in tiles_for_bbox(*box, z):
            (root / str(z) / str(x)).mkdir(parents=True, exist_ok=True)
            (root / str(z) / str(x) / f'{y}.png').write_bytes(PNG)
    (root / 'metadata.json').write_text(json.dumps({
        'name': 'test', 'center': list(MDRS), 'bounds': list(box), 'min_zoom': 12,
        'max_zoom': 17, 'attribution': 'test'}))
    return str(root)


@pytest.fixture(scope='module', autouse=True)
def _tile_env(tile_dir):
    old = os.environ.get('URC_TILE_DIR')
    os.environ['URC_TILE_DIR'] = tile_dir
    yield
    if old is None:
        os.environ.pop('URC_TILE_DIR')
    else:
        os.environ['URC_TILE_DIR'] = old


@pytest.fixture
def shell():
    ctl = MissionController(frame=LocalFrame(*MDRS))
    win = gui.OperatorWindow(FakeNode(), ctl)
    win.resize(1300, 800)
    win.show()
    QTest.qWait(500)
    yield win, ctl
    win.close()


def click_empty_map(win, lat, lon):
    win._map._map.runJavaScriptForMap(f'map.fire("click", {{latlng: L.latLng({lat}, {lon})}});')


def js(win, script):
    box = []
    win._map._map.getJsresponseForMap(script, lambda r: box.append(r))
    assert wait_for(lambda: box), script
    return box[0]


def test_click_on_empty_map_adds_selected_gnss_waypoint_with_next_name(shell):
    win, ctl = shell
    win._panel.set_add_mode(True)
    ctl.add_waypoint('Existing', MDRS[0] + 0.01, MDRS[1])        # far from the click
    click_empty_map(win, 38.4062, -110.7915)
    assert wait_for(lambda: len(ctl.model) == 2)
    wp = ctl.model.waypoints[-1]
    assert (wp.name, wp.lat_deg, wp.lon_deg) == ('WP2', 38.4062, -110.7915)
    assert wp.target_type is TargetType.GNSS
    assert ctl.selected_id == wp.id and win._map.selected_waypoint_id == wp.id
    assert 'Added WP2' in win._panel._status.text()


def test_new_waypoint_appears_as_a_marker_on_the_map(shell):
    win, ctl = shell
    win._panel.set_add_mode(True)
    assert len(win._map._markers) == 0
    click_empty_map(win, 38.4062, -110.7915)
    assert wait_for(lambda: len(ctl.model) == 1)
    # missionChanged -> map.set_waypoints drew a marker for it (and the panel lists it)
    assert wait_for(lambda: len(win._map._markers) == 1)
    assert win._panel._list.count() == 1
    marker = win._map._markers[ctl.model.waypoints[0].id]
    assert wait_for(lambda: js(win, f'{marker.layerName}.getLatLng().lat') == pytest.approx(38.4062))


def test_click_near_existing_waypoint_selects_it_instead_of_adding(shell):
    win, ctl = shell
    wp = ctl.add_waypoint('Alpha', 38.4058, -110.7919)
    zoom = js(win, 'map.getZoom()')
    px_deg = meters_per_pixel(MDRS[0], zoom) / 111_320
    click_empty_map(win, wp.lat_deg + 8 * px_deg, wp.lon_deg)     # 8 px away: inside the 20 px threshold
    assert wait_for(lambda: ctl.selected_id == wp.id)
    QTest.qWait(300)
    assert len(ctl.model) == 1


def test_out_of_range_click_is_rejected_with_a_message_and_no_crash(shell):
    win, ctl = shell
    win._panel.set_add_mode(True)
    click_empty_map(win, MDRS[0], MDRS[1] + 2.0)                  # ~175 km east: beyond the 100 km limit
    assert wait_for(lambda: 'not added' in win._panel._status.text())
    assert len(ctl.model) == 0 and win._map._markers == {}
    assert ctl.selected_id is None
    assert 'b71c1c' in win._panel._status.styleSheet()            # shown as an error, like panel errors


def test_impossible_coordinate_from_a_wrapped_map_is_rejected(shell):
    win, ctl = shell
    win._panel.set_add_mode(True)
    win._on_map_click_add(38.4, 190.0)                            # Leaflet can report lng > 180 when panned around the world
    win._on_map_click_add(95.0, -110.79)
    assert len(ctl.model) == 0
    assert 'not added' in win._panel._status.text()


def test_click_add_without_a_frame_still_works_and_names_continue(shell):
    win, ctl = shell
    win._panel.set_add_mode(True)
    ctl.set_frame(None)
    win._on_map_click_add(38.4061, -110.7913)
    win._panel.set_add_mode(True)               # one-shot: re-arm for the second
    win._on_map_click_add(38.4062, -110.7912)
    assert [w.name for w in ctl.model] == ['WP1', 'WP2']


# -- add mode (the toggle) -----------------------------------------------------------------------

def cursor(win):
    return js(win, 'getComputedStyle(map.getContainer()).cursor')


def test_add_mode_is_off_by_default_and_click_does_nothing_but_hint(shell):
    win, ctl = shell
    assert win._panel.add_mode is False and not win._panel._add_mode.isChecked()
    click_empty_map(win, 38.4062, -110.7915)
    assert wait_for(lambda: 'Add waypoint on click' in win._panel._status.text())
    QTest.qWait(200)
    assert len(ctl.model) == 0 and win._map._markers == {}
    assert win._panel.add_mode is False
    assert '616161' in win._panel._status.styleSheet()            # neutral hint, not an error


def test_add_mode_on_adds_as_before(shell):
    win, ctl = shell
    win._panel._add_mode.click()                                  # the operator presses the button
    assert win._panel.add_mode is True
    click_empty_map(win, 38.4062, -110.7915)
    assert wait_for(lambda: len(ctl.model) == 1)
    assert ctl.model.waypoints[0].name == 'WP1'


def test_add_mode_turns_itself_off_after_a_successful_add(shell):
    win, ctl = shell
    win._panel.set_add_mode(True)
    click_empty_map(win, 38.4062, -110.7915)
    assert wait_for(lambda: len(ctl.model) == 1)
    assert win._panel.add_mode is False and not win._panel._add_mode.isChecked()
    # A second click without re-arming does not add another waypoint.
    click_empty_map(win, 38.4070, -110.7900)
    assert wait_for(lambda: 'Add waypoint on click' in win._panel._status.text())
    QTest.qWait(200)
    assert len(ctl.model) == 1
    # Re-arming allows exactly one more.
    win._panel.set_add_mode(True)
    click_empty_map(win, 38.4070, -110.7900)
    assert wait_for(lambda: len(ctl.model) == 2)
    assert win._panel.add_mode is False


def test_add_mode_stays_on_after_a_rejected_add(shell):
    win, ctl = shell
    win._panel.set_add_mode(True)
    click_empty_map(win, MDRS[0], MDRS[1] + 2.0)                  # out of range: rejected
    assert wait_for(lambda: 'not added' in win._panel._status.text())
    assert win._panel.add_mode is True                            # still armed for a corrected click
    click_empty_map(win, 38.4062, -110.7915)
    assert wait_for(lambda: len(ctl.model) == 1)
    assert win._panel.add_mode is False


@pytest.mark.parametrize('armed', [False, True])
def test_selecting_an_existing_waypoint_works_in_either_mode(shell, armed):
    win, ctl = shell
    wp = ctl.add_waypoint('Alpha', 38.4058, -110.7919)
    ctl.select(None)
    win._panel.set_add_mode(armed)
    click_empty_map(win, wp.lat_deg, wp.lon_deg)                  # on top of the marker
    assert wait_for(lambda: ctl.selected_id == wp.id)
    QTest.qWait(200)
    assert len(ctl.model) == 1                                    # selected, never added
    assert win._panel.add_mode is armed                           # and the mode is untouched


def test_add_mode_is_visible_on_the_button_and_the_map_cursor(shell):
    win, ctl = shell
    button = win._panel._add_mode
    off_text, off_style = button.text(), button.styleSheet()
    assert cursor(win) != 'crosshair'
    win._panel.set_add_mode(True)
    assert wait_for(lambda: cursor(win) == 'crosshair')
    assert button.isChecked() and 'ff9800' in button.styleSheet() and 'Click the map' in button.text()
    win._panel.set_add_mode(False)
    assert wait_for(lambda: cursor(win) != 'crosshair')
    assert (button.text(), button.styleSheet()) == (off_text, off_style)


def test_cursor_resets_after_a_successful_add(shell):
    win, ctl = shell
    win._panel.set_add_mode(True)
    assert wait_for(lambda: cursor(win) == 'crosshair')
    click_empty_map(win, 38.4062, -110.7915)
    assert wait_for(lambda: len(ctl.model) == 1)
    assert wait_for(lambda: cursor(win) != 'crosshair')
