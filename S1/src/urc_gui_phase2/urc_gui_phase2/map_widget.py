"""Offline Leaflet map widget (pyqtlet2 + QtWebEngine). No live network requests.

Public interface (everything else is private):

    OfflineMapWidget(tile_dir=None, threshold_px=20)
        set_waypoints(waypoints)         # mission_model.Waypoint items -> markers
        select_waypoint(waypoint_id)     # programmatic selection, or None to clear
        selected_waypoint_id             # property
        set_rover_position(lat, lon)     # WGS84 -> distinct rover marker
        set_rover_path(points)           # accepted WGS84 history -> red polyline
        set_rover_heading(degrees)       # compass heading -> short direction line
        set_obstacle_enu(east, north, radius, clearance)
        set_planned_route_enu(points)    # remaining ENU planner route
        clear_rover_track()              # clear path + heading, keep rover marker
        set_crosshair_cursor(enabled)    # crosshair over the map while placing a waypoint
        set_local_frame(frame)           # coordinate_convert.LocalFrame
        set_rover_enu(east_m, north_m)   # local ENU -> WGS84 via the frame
        clear_rover()
        selectionChanged(object)         # signal: waypoint id, or None if cleared
        mapClickedForNewWaypoint(float, float)  # signal: lat, lon of a click on empty map
        blocked_requests                 # property: non-local URLs the engine refused

Dependencies on the rest of the project are deliberately only Waypoint/Status
from mission_model and LocalFrame.to_wgs84 from coordinate_convert (both used
by duck typing), plus the Qt-free tile modules. No ROS, no event loop of its
own: the host application owns QApplication and its ROS timer.

How "no network" is enforced (three independent layers):
  1. The tile URL template is file://, so no tile is ever *asked for* over the
     network. Leaflet/QWebChannel scripts are the copies bundled inside
     pyqtlet2 (file:// and qrc://), not CDN links.
  2. A QWebEngineUrlRequestInterceptor allow-lists file/qrc/data/blob/about
     and blocks everything else, recording each blocked URL. If anything ever
     tries http(s), it fails and shows up in `blocked_requests`.
  3. tools/verify_offline.py runs the whole thing inside an empty network
     namespace with DNS disabled, which is the demonstration for judges.
"""

import html
import json
import math
import threading
from typing import Iterable, List, Optional
from urllib.parse import quote

from pyqtlet2 import L, MapWidget
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWebEngineCore import QWebEngineUrlRequestInterceptor
from PyQt5.QtWebEngineWidgets import QWebEngineProfile
from PyQt5.QtWidgets import QLabel, QVBoxLayout, QWidget

from urc_gui_phase2.map_geometry import pick_nearest
from urc_gui_phase2.mission_model import Status, Waypoint
from urc_gui_phase2.offline_tiles import (
    OfflineTileSource, TileSourceError, default_tile_dir, is_local_url)

# Phase 1 challenge doc: MDRS approximately 38.4058 N, 110.7919 W.
MDRS_CENTER = (38.4058, -110.7919)
DEFAULT_ZOOM = 15
DEFAULT_THRESHOLD_PX = 20

STATUS_COLORS = {Status.PENDING: '#1e88e5', Status.ACTIVE: '#fb8c00', Status.DONE: '#43a047'}
SELECTED_OUTLINE = '#ffd600'
ROVER_COLOR = '#d50000'

# Shown by Leaflet in place of any tile missing from disk, so a gap in the
# offline set is obvious rather than a silent grey square.
_MISSING_TILE_SVG = (
    '<svg xmlns="http://www.w3.org/2000/svg" width="256" height="256">'
    '<rect width="256" height="256" fill="#eeeeee"/>'
    '<path d="M0 0L256 256M256 0L0 256" stroke="#cccccc" stroke-width="2"/>'
    '<text x="128" y="132" font-size="16" text-anchor="middle" fill="#888888">'
    'no offline tile</text></svg>')
_MISSING_TILE_URL = 'data:image/svg+xml;charset=utf-8,' + quote(_MISSING_TILE_SVG, safe='')


class OfflineOnlyInterceptor(QWebEngineUrlRequestInterceptor):
    """Blocks every request that is not to a local resource, and records it.

    interceptRequest runs on Chromium's IO thread, not the Qt main thread, so
    the record is guarded by a lock and no Qt objects are touched from it.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._lock = threading.Lock()
        self._blocked: List[str] = []
        self._allowed_count = 0

    def interceptRequest(self, info):  # noqa: N802 (Qt naming)
        url = info.requestUrl().toString()
        if is_local_url(url):
            with self._lock:
                self._allowed_count += 1
        else:
            info.block(True)
            with self._lock:
                self._blocked.append(url)

    @property
    def blocked(self) -> List[str]:
        with self._lock:
            return list(self._blocked)

    @property
    def allowed_count(self) -> int:
        with self._lock:
            return self._allowed_count


def _js_string_body(text: str) -> str:
    """Escape user text for placement between double quotes in generated JS.

    pyqtlet2 splices tooltip text into a JS string literal, and Leaflet renders
    tooltips as HTML, so waypoint names must be HTML-escaped and JS-escaped.
    """
    return json.dumps(html.escape(text))[1:-1]


class OfflineMapWidget(QWidget):
    selectionChanged = pyqtSignal(object)  # waypoint id (str) or None
    # A click that hit no waypoint: WGS84 lat, lon as computed by Leaflet from the click pixel.
    # The widget only reports it, always; whether a waypoint is created (e.g. only while an
    # "add waypoint" mode is armed) is the host's decision.
    mapClickedForNewWaypoint = pyqtSignal(float, float)

    def __init__(self, tile_dir: Optional[str] = None, threshold_px: float = DEFAULT_THRESHOLD_PX,
                 parent=None):
        super().__init__(parent)
        self._threshold_px = threshold_px
        self._waypoints: List[Waypoint] = []
        self._markers = {}            # waypoint id -> pyqtlet2 CircleMarker
        self._selected_id: Optional[str] = None
        self._rover = None            # pyqtlet2 CircleMarker or None
        self._rover_latlon = None
        self._rover_path = None       # Leaflet Polyline or None
        self._heading_line = None     # short line showing forward direction
        self._planned_route = None    # remaining obstacle-planner route
        self._obstacles = []          # physical obstacle circles
        self._obstacle_safeties = []  # inflated planner boundaries
        self._obstacle_spec = None
        self._frame = None
        self._last_zoom = float(DEFAULT_ZOOM)

        # Load the tile source first; a missing tile set is reported in the
        # banner and the map still runs (markers on a blank background).
        self._source: Optional[OfflineTileSource] = None
        self._source_error = ''
        try:
            self._source = OfflineTileSource(tile_dir or default_tile_dir())
        except TileSourceError as exc:
            self._source_error = str(exc)

        # The interceptor must be on the profile *before* the page loads, and
        # pyqtlet2's MapWidget loads its page inside its constructor. It uses
        # the default profile, so install it there (this also covers any other
        # web view in the process, which is the safe direction for this rule).
        self._interceptor = OfflineOnlyInterceptor(self)
        QWebEngineProfile.defaultProfile().setUrlRequestInterceptor(self._interceptor)

        self._banner = QLabel()
        self._banner.setWordWrap(True)
        self._map_view = MapWidget()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self._banner)
        layout.addWidget(self._map_view, 1)

        center = (self._source.center if self._source and self._source.center else MDRS_CENTER)
        map_options = {}
        zoom = DEFAULT_ZOOM
        if self._source:
            map_options = {'minZoom': self._source.min_zoom, 'maxZoom': self._source.max_zoom}
            zoom = max(self._source.min_zoom, min(self._source.max_zoom, DEFAULT_ZOOM))
        self._map = L.map(self._map_view, map_options)
        self._map.setView(list(center), zoom)
        self._last_zoom = float(zoom)

        if self._source:
            tile_options = {
                'minZoom': self._source.min_zoom,
                'maxZoom': self._source.max_zoom,
                'errorTileUrl': _MISSING_TILE_URL,
                'attribution': self._source.attribution,
            }
            if self._source.bounds:
                s, w, n, e = self._source.bounds
                tile_options['bounds'] = [[s, w], [n, e]]
            self._tiles = L.tileLayer(self._source.url_template, tile_options)
            self._map.addLayer(self._tiles)

        # Direct marker click events are unreliable in pyqtlet2, so selection
        # is done from the *map* click plus nearest-marker-within-threshold.
        self._map.clicked.connect(self._on_map_clicked)
        self._refresh_banner()

    # -- public API ----------------------------------------------------------

    @property
    def selected_waypoint_id(self) -> Optional[str]:
        return self._selected_id

    @property
    def blocked_requests(self) -> List[str]:
        return self._interceptor.blocked

    @property
    def tile_source(self) -> Optional[OfflineTileSource]:
        return self._source

    def set_waypoints(self, waypoints: Iterable[Waypoint]) -> None:
        """Replace all waypoint markers. Keeps the selection if that id still exists."""
        self._waypoints = list(waypoints)
        if self._selected_id not in {w.id for w in self._waypoints}:
            self._set_selected(None)
        self._redraw_waypoints()
        self._refresh_banner()

    def select_waypoint(self, waypoint_id: Optional[str]) -> None:
        if waypoint_id is not None and waypoint_id not in {w.id for w in self._waypoints}:
            raise KeyError(f'no waypoint with id {waypoint_id!r} on the map')
        self._set_selected(waypoint_id)

    def set_rover_position(self, lat_deg: float, lon_deg: float) -> None:
        latlon = (float(lat_deg), float(lon_deg))
        if latlon == self._rover_latlon:
            return
        self._rover_latlon = latlon
        if self._rover is None:
            self._rover = L.circleMarker(list(latlon), {
                'radius': 11, 'color': '#ffffff', 'weight': 3,
                'fillColor': ROVER_COLOR, 'fillOpacity': 1.0})
            self._map.addLayer(self._rover)
            self._rover.bindTooltip('Rover')
        else:
            self._map.runJavaScriptForMap(
                f'{self._rover.layerName}.setLatLng([{latlon[0]!r}, {latlon[1]!r}]);')
        self._refresh_banner()

    def set_rover_path(self, points: Iterable) -> None:
        """Draw the accepted WGS 84 position history as one offline polyline."""
        latlngs = [[float(lat), float(lon)] for lat, lon in points]
        if len(latlngs) < 2:
            if self._rover_path is not None:
                self._map.removeLayer(self._rover_path)
                self._rover_path = None
            return

        encoded = json.dumps(latlngs)
        if self._rover_path is None:
            self._rover_path = L.polyline(latlngs, {
                'color': '#dc2626', 'weight': 4, 'opacity': 0.8,
            })
            self._map.addLayer(self._rover_path)
            self._rover_path.bindTooltip('Traveled path')
        else:
            self._map.runJavaScriptForMap(
                f'{self._rover_path.layerName}.setLatLngs({encoded});'
            )
        self._map.runJavaScriptForMap(
            f'{self._rover_path.layerName}.bringToBack();'
        )
        self._bring_rover_to_front()

    def set_rover_heading(self, heading_deg: float, length_m: float = 6.0) -> None:
        """Show heading clockwise from north as a short line from the rover."""
        heading_deg, length_m = float(heading_deg), float(length_m)
        if not (math.isfinite(heading_deg) and math.isfinite(length_m) and length_m > 0.0):
            raise ValueError(
                'heading and indicator length must be finite; length must be positive'
            )
        if self._frame is None or self._rover_latlon is None:
            raise RuntimeError('local frame and rover position are required before heading')

        rover_enu = self._frame.to_enu(*self._rover_latlon)
        angle = math.radians(heading_deg)
        tip = self._frame.to_wgs84(
            rover_enu.east_m + math.sin(angle) * length_m,
            rover_enu.north_m + math.cos(angle) * length_m,
        )
        latlngs = [list(self._rover_latlon), [tip.lat_deg, tip.lon_deg]]
        encoded = json.dumps(latlngs)

        if self._heading_line is None:
            self._heading_line = L.polyline(latlngs, {
                'color': '#111827', 'weight': 5, 'opacity': 1.0,
            })
            self._map.addLayer(self._heading_line)
            self._heading_line.bindTooltip(f'Heading {heading_deg:.0f} degrees')
        else:
            self._map.runJavaScriptForMap(
                f'{self._heading_line.layerName}.setLatLngs({encoded});'
                f'{self._heading_line.layerName}.setTooltipContent('
                f'{json.dumps(f"Heading {heading_deg:.0f} degrees")});'
            )
        self._bring_rover_to_front()

    def clear_rover_track(self) -> None:
        """Remove traveled path and heading while keeping the rover marker."""
        for layer_name in ('_rover_path', '_heading_line'):
            layer = getattr(self, layer_name)
            if layer is not None:
                self._map.removeLayer(layer)
                setattr(self, layer_name, None)

    def set_obstacle_enu(self, east_m: float, north_m: float,
                         radius_m: float, clearance_m: float = 0.0) -> None:
        """Draw a physical obstacle and its inflated ENU safety boundary."""
        self.set_obstacles_enu([{
            'east_m': east_m,
            'north_m': north_m,
            'radius_m': radius_m,
            'clearance_m': clearance_m,
        }])

    def set_obstacles_enu(self, obstacles: Iterable) -> None:
        """Draw every physical obstacle and inflated safety boundary."""
        normalized = []
        for obstacle in obstacles:
            if isinstance(obstacle, dict):
                values = tuple(float(obstacle[name]) for name in (
                    'east_m', 'north_m', 'radius_m', 'clearance_m'))
            else:
                values = tuple(float(value) for value in obstacle)
            if len(values) != 4:
                raise ValueError('each obstacle requires four values')
            east_m, north_m, radius_m, clearance_m = values
            if not all(math.isfinite(value) for value in values):
                raise ValueError('obstacle values must be finite')
            if radius_m <= 0.0 or clearance_m < 0.0:
                raise ValueError(
                    'obstacle radius must be positive and clearance non-negative'
                )
            normalized.append(values)

        spec = tuple(normalized)
        if spec == self._obstacle_spec:
            return
        if self._frame is None:
            raise RuntimeError('local frame is required before drawing obstacles')

        self.clear_obstacle()
        for index, values in enumerate(normalized, start=1):
            east_m, north_m, radius_m, clearance_m = values
            center = self._frame.to_wgs84(east_m, north_m)
            latlng = [center.lat_deg, center.lon_deg]
            safety = L.circle(latlng, {
                'radius': radius_m + clearance_m,
                'color': '#f59e0b',
                'weight': 2,
                'dashArray': '6 5',
                'fillOpacity': 0.08,
            })
            physical = L.circle(latlng, {
                'radius': radius_m,
                'color': '#7f1d1d',
                'weight': 3,
                'fillColor': '#dc2626',
                'fillOpacity': 0.45,
            })
            self._map.addLayer(safety)
            self._map.addLayer(physical)
            physical.bindTooltip(f'Simulated obstacle {index}')
            safety.bindTooltip(f'Obstacle {index} safety boundary')
            self._obstacle_safeties.append(safety)
            self._obstacles.append(physical)
        self._obstacle_spec = spec

    def clear_obstacle(self) -> None:
        """Remove all obstacles and safety boundaries from the map."""
        for layers in (self._obstacles, self._obstacle_safeties):
            for layer in layers:
                self._map.removeLayer(layer)
            layers.clear()
        self._obstacle_spec = None

    def set_planned_route_enu(self, points: Iterable) -> None:
        """Draw the remaining ENU route, including obstacle detours."""
        if self._frame is None:
            raise RuntimeError('local frame is required before drawing a route')
        geographic_points = []
        if self._rover_latlon is not None:
            geographic_points.append(list(self._rover_latlon))
        for east_m, north_m in points:
            position = self._frame.to_wgs84(float(east_m), float(north_m))
            geographic_points.append([position.lat_deg, position.lon_deg])

        if len(geographic_points) < 2:
            if self._planned_route is not None:
                self._map.removeLayer(self._planned_route)
                self._planned_route = None
            return

        encoded = json.dumps(geographic_points)
        if self._planned_route is None:
            self._planned_route = L.polyline(geographic_points, {
                'color': '#2563eb',
                'weight': 3,
                'opacity': 0.9,
                'dashArray': '8 6',
            })
            self._map.addLayer(self._planned_route)
            self._planned_route.bindTooltip('Planned route')
        else:
            self._map.runJavaScriptForMap(
                f'{self._planned_route.layerName}.setLatLngs({encoded});'
            )
        self._map.runJavaScriptForMap(
            f'{self._planned_route.layerName}.bringToBack();'
        )
        self._bring_rover_to_front()

    def set_crosshair_cursor(self, enabled: bool) -> None:
        """Crosshair over the map (True) or Leaflet's default grab cursor (False)."""
        cursor = "'crosshair'" if enabled else "''"
        self._map.runJavaScriptForMap(f'{self._map.jsName}.getContainer().style.cursor = {cursor};')

    def set_local_frame(self, frame) -> None:
        """The LocalFrame used to convert ENU rover positions to WGS84."""
        self._frame = frame

    def set_rover_enu(self, east_m: float, north_m: float) -> None:
        """Rover position in the local frame. Raises if no frame is set, or on FrameRangeError."""
        if self._frame is None:
            raise RuntimeError('set_local_frame() must be called before set_rover_enu()')
        geo = self._frame.to_wgs84(east_m, north_m)
        self.set_rover_position(geo.lat_deg, geo.lon_deg)

    def clear_rover(self) -> None:
        if self._rover is not None:
            self._map.removeLayer(self._rover)
            self._rover = None
        self._rover_latlon = None
        if self._heading_line is not None:
            self._map.removeLayer(self._heading_line)
            self._heading_line = None
        self._refresh_banner()

    # -- selection -----------------------------------------------------------

    def _on_map_clicked(self, event: dict) -> None:
        latlng = event.get('latlng') or {}
        try:
            lat, lng = float(latlng['lat']), float(latlng['lng'])
        except (KeyError, TypeError, ValueError):
            return
        # Ask Leaflet for the zoom at click time: the pixel threshold depends on it.
        self._map.getZoom(lambda zoom: self._select_at(lat, lng, zoom))

    def _select_at(self, lat: float, lng: float, zoom) -> None:
        if isinstance(zoom, (int, float)):
            self._last_zoom = float(zoom)
        hit = pick_nearest(lat, lng, [(w.id, w.lat_deg, w.lon_deg) for w in self._waypoints],
                           self._last_zoom, self._threshold_px)
        if hit is None:
            # Empty map: ask the host to add a waypoint here. The selection is left alone;
            # the host selects the new waypoint if the add succeeds.
            self.mapClickedForNewWaypoint.emit(lat, lng)
            return
        self._set_selected(hit)

    def _set_selected(self, waypoint_id: Optional[str]) -> None:
        if waypoint_id == self._selected_id:
            return
        self._selected_id = waypoint_id
        self._redraw_waypoints()
        self.selectionChanged.emit(waypoint_id)

    # -- drawing -------------------------------------------------------------

    def _redraw_waypoints(self) -> None:
        for marker in self._markers.values():
            self._map.removeLayer(marker)
        self._markers = {}
        for wp in self._waypoints:
            selected = wp.id == self._selected_id
            marker = L.circleMarker([wp.lat_deg, wp.lon_deg], {
                'radius': 10 if selected else 7,
                'color': SELECTED_OUTLINE if selected else '#212121',
                'weight': 4 if selected else 2,
                'fillColor': STATUS_COLORS[wp.status],
                'fillOpacity': 0.9})
            self._map.addLayer(marker)
            marker.bindTooltip(_js_string_body(wp.name))
            self._markers[wp.id] = marker
        self._bring_rover_to_front()

    def _bring_rover_to_front(self) -> None:
        """Keep the heading and rover visible above paths and waypoint markers."""
        if self._heading_line is not None:
            self._map.runJavaScriptForMap(
                f'{self._heading_line.layerName}.bringToFront();'
            )
        if self._rover is not None:
            self._map.runJavaScriptForMap(
                f'{self._rover.layerName}.bringToFront();'
            )

    def _refresh_banner(self) -> None:
        if self._source is None:
            self._set_banner(f'NO OFFLINE MAP TILES: {self._source_error}', ok=False)
            return
        points = [(w.lat_deg, w.lon_deg) for w in self._waypoints]
        if self._rover_latlon:
            points.append(self._rover_latlon)
        outside = sum(1 for lat, lon in points if not self._source.covers(lat, lon))
        if outside:
            self._set_banner(f'{outside} marker(s) lie outside the offline tile coverage '
                             f'({self._source.describe()}); the background is blank there.', ok=False)
        else:
            self._set_banner(f'Offline map: {self._source.describe()}', ok=True)

    def _set_banner(self, text: str, ok: bool) -> None:
        self._banner.setText(text)
        self._banner.setStyleSheet(
            'padding: 3px 6px; background: %s; color: %s;' %
            (('#e8f5e9', '#1b5e20') if ok else ('#ffebee', '#b71c1c')))
