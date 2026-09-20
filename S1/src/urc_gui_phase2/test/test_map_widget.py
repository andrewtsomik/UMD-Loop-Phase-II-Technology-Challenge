"""Widget tests run a real QtWebEngine page offscreen (no display needed)."""
import json
import math
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
from urc_gui_phase2.map_geometry import (  # noqa: E402
    _mercator_pixels, bbox_around, meters_per_pixel, tile_bounds, tiles_for_bbox)
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
    seen, added = [], []
    widget.selectionChanged.connect(seen.append)
    widget.mapClickedForNewWaypoint.connect(lambda lat, lon: added.append((lat, lon)))
    zoom = js(widget, 'map.getZoom()')
    px = meters_per_pixel(MDRS[0], zoom) / 111_320  # degrees of latitude per pixel

    fire_click(widget, alpha.lat_deg + 8 * px, alpha.lon_deg)          # 8 px away -> hit
    assert wait_for(lambda: widget.selected_waypoint_id == alpha.id)
    fire_click(widget, bravo.lat_deg, bravo.lon_deg + 1e-6)           # on Bravo
    assert wait_for(lambda: widget.selected_waypoint_id == bravo.id)
    assert added == []                                                # hits never propose an add
    fire_click(widget, alpha.lat_deg + 200 * px, alpha.lon_deg)       # empty map -> add request
    assert wait_for(lambda: len(added) == 1)
    assert widget.selected_waypoint_id == bravo.id                    # selection untouched
    assert seen == [alpha.id, bravo.id]
    widget.select_waypoint(None)


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


def test_rover_path_heading_and_reset(widget):
    frame = LocalFrame(*MDRS)
    second = frame.to_wgs84(10.0, 0.0)
    widget.set_local_frame(frame)
    widget.set_rover_position(second.lat_deg, second.lon_deg)
    widget.set_rover_path([MDRS, (second.lat_deg, second.lon_deg)])
    widget.set_rover_heading(90.0)

    assert widget._rover_path is not None
    assert widget._heading_line is not None

    widget.clear_rover_track()
    assert widget._rover_path is None
    assert widget._heading_line is None
    assert widget._rover is not None
    widget.clear_rover()


def test_obstacle_and_planned_detour_layers(widget):
    frame = LocalFrame(*MDRS)
    widget.set_local_frame(frame)
    widget.set_rover_position(*MDRS)

    widget.set_obstacle_enu(15.0, 0.0, 3.0, 2.0)
    widget.set_planned_route_enu([(15.0, 6.0), (30.0, 0.0)])

    assert widget._obstacle is not None
    assert widget._obstacle_safety is not None
    assert widget._planned_route is not None

    widget.set_planned_route_enu([])
    widget.clear_obstacle()
    assert widget._planned_route is None
    assert widget._obstacle is None
    assert widget._obstacle_safety is None
    widget.clear_rover()


def test_marker_outside_coverage_is_flagged_in_banner(widget, mission):
    widget.set_waypoints(mission.waypoints)
    widget.set_rover_position(MDRS[0] + 1.0, MDRS[1])
    assert '1 marker(s) lie outside' in widget._banner.text()
    widget.clear_rover()
    assert 'Offline map' in widget._banner.text()


# -- click on empty map -> mapClickedForNewWaypoint ---------------------------------------------

def dom_click(widget, dx, dy):
    """A real DOM click about `dx` px right / `dy` px down of the map container's centre.

    Unlike fire_click, this goes through Leaflet's own pixel -> lat/lon conversion, which is the
    conversion that decides where a new waypoint lands on the ground. MouseEvent coordinates are
    integers, so it returns the offset (from the exact centre) that was really clicked.
    """
    return js(widget,
              '(function() { var c = map.getContainer(), r = c.getBoundingClientRect(),'
              ' s = map.getSize(), x = Math.round(r.left + s.x / 2 + ' + str(dx) + '),'
              ' y = Math.round(r.top + s.y / 2 + ' + str(dy) + ');'
              ' c.dispatchEvent(new MouseEvent("click", {bubbles: true, clientX: x, clientY: y}));'
              ' return [x - r.left - s.x / 2, y - r.top - s.y / 2]; })()')


def latlon_from_world_pixels(px, py, zoom):
    """Independent inverse Web Mercator (the standard slippy-map formulas)."""
    scale = 256 * 2.0 ** zoom
    lon = px / scale * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * py / scale))))
    return lat, lon


def collect_adds(widget):
    added = []
    widget.mapClickedForNewWaypoint.connect(lambda lat, lon: added.append((lat, lon)))
    return added


def test_independent_inverse_matches_library_forward_and_tile_corners():
    # Guards the reference used below: it must invert map_geometry's forward transform
    # and reproduce the known corners of a real tile exactly.
    for z in (12, 15, 17):
        px, py = _mercator_pixels(*MDRS, z)
        lat, lon = latlon_from_world_pixels(px, py, z)
        assert (lat, lon) == pytest.approx(MDRS, abs=1e-9)
    south, west, north, east = tile_bounds(6299, 12592, 15)  # the tile holding MDRS at z15
    lat, lon = latlon_from_world_pixels(6299 * 256, 12592 * 256, 15)
    assert (lat, lon) == pytest.approx((north, west), abs=1e-9)
    lat, lon = latlon_from_world_pixels(6300 * 256, 12593 * 256, 15)
    assert (lat, lon) == pytest.approx((south, east), abs=1e-9)


def test_click_at_map_centre_reports_the_known_centre_point(widget):
    widget.set_waypoints([])
    js(widget, f'map.setView([{MDRS[0]}, {MDRS[1]}], 15, {{animate: false}}); 1')
    added = collect_adds(widget)
    adx, ady = dom_click(widget, 0, 0)
    assert wait_for(lambda: added)
    # Known point: the map was centred on MDRS. Leaflet snaps its pixel origin to whole pixels,
    # so its centre can be up to half a pixel from the requested one; anything larger is a bug.
    half_pixel_deg = 0.5 * meters_per_pixel(MDRS[0], 15) / 111_320 * 1.01
    assert added[0][0] == pytest.approx(MDRS[0], abs=half_pixel_deg + 1e-9)
    assert added[0][1] == pytest.approx(MDRS[1], abs=half_pixel_deg / math.cos(math.radians(MDRS[0])))
    # Precisely: the click equals independent math from Leaflet's actual centre and clicked pixel.
    c = js(widget, 'map.getCenter()')
    cx, cy = _mercator_pixels(c['lat'], c['lng'], 15)
    assert added[0] == pytest.approx(latlon_from_world_pixels(cx + adx, cy + ady, 15), abs=1e-7)


@pytest.mark.parametrize('dx,dy', [(120, -80), (-300, 200), (250, 240), (-5, -5)])
def test_click_pixel_converts_to_the_same_lat_lon_as_independent_mercator_math(widget, dx, dy):
    widget.set_waypoints([])
    added = collect_adds(widget)
    zoom = js(widget, 'map.getZoom()')
    c = js(widget, 'map.getCenter()')
    cx, cy = _mercator_pixels(c['lat'], c['lng'], zoom)
    adx, ady = dom_click(widget, dx, dy)
    assert wait_for(lambda: added)
    expected = latlon_from_world_pixels(cx + adx, cy + ady, zoom)
    assert added[0] == pytest.approx(expected, abs=1e-7)   # 1e-7 deg ~ 1 cm


def test_click_offset_is_the_right_number_of_metres_on_the_ground(widget):
    """Pixels -> lat/lon -> LocalFrame ENU, checked against ground metres per pixel."""
    widget.set_waypoints([])
    added = collect_adds(widget)
    zoom = js(widget, 'map.getZoom()')
    c = js(widget, 'map.getCenter()')
    frame = LocalFrame(c['lat'], c['lng'])
    adx, ady = dom_click(widget, 200, -150)   # about 200 px east, 150 px north of the centre
    assert wait_for(lambda: added)
    enu = frame.to_enu(*added[0])
    mpp = meters_per_pixel(c['lat'], zoom)
    # UTM (ellipsoid) vs Web Mercator (sphere) differ by well under 1% at this scale.
    assert enu.east_m == pytest.approx(adx * mpp, rel=0.01)
    assert enu.north_m == pytest.approx(-ady * mpp, rel=0.01)


def test_click_on_empty_map_does_not_touch_selection_or_emit_selection(widget, mission):
    widget.set_waypoints(mission.waypoints)
    widget.select_waypoint(mission.waypoints[0].id)
    added, seen = collect_adds(widget), []
    widget.selectionChanged.connect(seen.append)
    dom_click(widget, 300, 200)           # far from both waypoints
    assert wait_for(lambda: added)
    assert widget.selected_waypoint_id == mission.waypoints[0].id and seen == []
    widget.select_waypoint(None)
