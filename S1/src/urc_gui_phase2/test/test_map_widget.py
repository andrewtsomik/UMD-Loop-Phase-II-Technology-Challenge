"""Widget tests run a real QtWebEngine page offscreen (no display needed)."""
import json
import os
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('QTWEBENGINE_CHROMIUM_FLAGS', '--no-sandbox --disable-gpu')

import pytest

pytest.importorskip('PyQt5.QtWebEngineWidgets')
pytest.importorskip('pyqtlet2')

from PyQt5.QtTest import QTest  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

from urc_gui_phase2.coordinate_convert import LocalFrame  # noqa: E402
from urc_gui_phase2.map_geometry import bbox_around, meters_per_pixel, tiles_for_bbox  # noqa: E402
from urc_gui_phase2.map_widget import OfflineMapWidget  # noqa: E402
from urc_gui_phase2.mission_model import MissionModel, Status  # noqa: E402

MDRS = (38.4058, -110.7919)
# 1x1 PNG
PNG = bytes.fromhex('89504e470d0a1a0a0000000d49484452000000010000000108060000001f15c489'
                    '0000000d49444154789c6360f8cfc0f00f0001040100e0c4fc4d0000000049454e44ae426082')


def wait_for(predicate, timeout=10.0):
    end = time.time() + timeout
    while time.time() < end:
        if predicate():
            return True
        QTest.qWait(50)
    return predicate()


def js(widget, script, timeout=10.0):
    box = []
    widget._map.getJsresponseForMap(script, lambda r: box.append(r))
    assert wait_for(lambda: box, timeout), f'no JS response for {script}'
    return box[0]


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture(scope='module')
def tile_dir(tmp_path_factory):
    root = tmp_path_factory.mktemp('tiles') / 'dir with space'
    box = bbox_around(*MDRS, 2500)
    for z in range(12, 18):
        for x, y in tiles_for_bbox(*box, z):
            (root / str(z) / str(x)).mkdir(parents=True, exist_ok=True)
            (root / str(z) / str(x) / f'{y}.png').write_bytes(PNG)
    (root / 'metadata.json').write_text(json.dumps({
        'name': 'test', 'center': list(MDRS), 'bounds': list(box), 'min_zoom': 12,
        'max_zoom': 17, 'attribution': 'test'}))
    return str(root)


@pytest.fixture(scope='module')
def widget(app, tile_dir):
    w = OfflineMapWidget(tile_dir)
    w.resize(800, 600)
    w.show()
    yield w
    w.close()


@pytest.fixture
def mission():
    m = MissionModel('t')
    m.add('Alpha', MDRS[0] + 0.0005, MDRS[1])
    m.add('Bravo "quoted" <b>x</b>', MDRS[0] - 0.002, MDRS[1] + 0.002)
    return m


def fire_click(widget, lat, lng):
    widget._map.runJavaScriptForMap(f'map.fire("click", {{latlng: L.latLng({lat}, {lng})}});')


def test_tiles_load_from_local_files_and_nothing_hits_the_network(widget):
    assert wait_for(lambda: js(widget, 'document.querySelectorAll("img.leaflet-tile-loaded").length') > 0)
    assert widget._interceptor.allowed_count > 0
    assert widget.blocked_requests == []
    urls = js(widget, 'Array.from(document.querySelectorAll("img.leaflet-tile")).map(i => i.src)')
    assert urls and all(u.startswith('file:///') for u in urls)
    assert 'Offline map' in widget._banner.text()


def test_interceptor_actually_blocks_network_requests(widget):
    js(widget, 'new Image().src = "http://tiles.example.invalid/1/2/3.png"; 1')
    assert wait_for(lambda: any('tiles.example.invalid' in u for u in widget.blocked_requests))


def test_click_selects_nearest_waypoint_within_threshold(widget, mission):
    widget.set_waypoints(mission.waypoints)
    alpha, bravo = mission.waypoints
    seen = []
    widget.selectionChanged.connect(seen.append)
    zoom = js(widget, 'map.getZoom()')
    px = meters_per_pixel(MDRS[0], zoom) / 111_320  # degrees of latitude per pixel

    fire_click(widget, alpha.lat_deg + 8 * px, alpha.lon_deg)          # 8 px away -> hit
    assert wait_for(lambda: widget.selected_waypoint_id == alpha.id)
    fire_click(widget, bravo.lat_deg, bravo.lon_deg + 1e-6)           # on Bravo
    assert wait_for(lambda: widget.selected_waypoint_id == bravo.id)
    fire_click(widget, alpha.lat_deg + 200 * px, alpha.lon_deg)       # empty map -> clears
    assert wait_for(lambda: widget.selected_waypoint_id is None)
    assert seen == [alpha.id, bravo.id, None]


def test_programmatic_select_and_unknown_id(widget, mission):
    widget.set_waypoints(mission.waypoints)
    widget.select_waypoint(mission.waypoints[0].id)
    assert widget.selected_waypoint_id == mission.waypoints[0].id
    with pytest.raises(KeyError):
        widget.select_waypoint('nope')
    widget.select_waypoint(None)


def test_selection_dropped_when_waypoint_removed(widget, mission):
    widget.set_waypoints(mission.waypoints)
    widget.select_waypoint(mission.waypoints[1].id)
    widget.set_waypoints(mission.waypoints[:1])
    assert widget.selected_waypoint_id is None


def test_markers_rendered_and_hostile_name_does_not_break_js(widget, mission):
    mission.set_status(mission.waypoints[0].id, Status.ACTIVE)
    widget.set_waypoints(mission.waypoints)
    assert wait_for(lambda: js(widget, 'document.querySelectorAll("path.leaflet-interactive").length') == 2)
    tooltips = js(widget, 'var out=[]; map.eachLayer(l => { if (l.getTooltip && l.getTooltip()) '
                          'out.push(l.getTooltip().getContent()); }); out')
    assert 'Bravo &quot;quoted&quot; &lt;b&gt;x&lt;/b&gt;' in tooltips  # rendered as text, not HTML


def test_rover_marker_is_distinct_and_moves_without_duplicating(widget, mission):
    widget.set_waypoints(mission.waypoints)
    widget.set_rover_position(*MDRS)
    widget.set_rover_position(MDRS[0] + 0.001, MDRS[1])
    assert wait_for(lambda: js(widget, 'document.querySelectorAll("path.leaflet-interactive").length') == 3)
    fills = js(widget, 'Array.from(document.querySelectorAll("path.leaflet-interactive")).map(p => p.getAttribute("fill"))')
    assert fills.count('#d50000') == 1 and len(fills) == 3  # one rover, two waypoints, none blue-on-red


def test_rover_from_local_frame(widget):
    with pytest.raises(RuntimeError):
        widget.set_rover_enu(1, 1)
    frame = LocalFrame(*MDRS)
    widget.set_local_frame(frame)
    widget.set_rover_enu(100.0, 50.0)
    geo = frame.to_wgs84(100.0, 50.0)
    assert widget._rover_latlon == pytest.approx((geo.lat_deg, geo.lon_deg))
    widget.clear_rover()
    assert widget._rover is None


def test_marker_outside_coverage_is_flagged_in_banner(widget, mission):
    widget.set_waypoints(mission.waypoints)
    widget.set_rover_position(MDRS[0] + 1.0, MDRS[1])
    assert '1 marker(s) lie outside' in widget._banner.text()
    widget.clear_rover()
    assert 'Offline map' in widget._banner.text()
