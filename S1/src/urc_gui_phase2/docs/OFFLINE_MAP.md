# Offline map (S1: MDRS area, no live network)

## One-time setup (needs internet, run once)

```bash
cd ~/ros2_ws/src/urc_gui_phase2
python3 tools/tile_downloader.py --dry-run   # shows tile counts, downloads nothing
python3 tools/tile_downloader.py             # -> tiles/mdrs/{z}/{x}/{y}.png + metadata.json
```

Default: 5 x 5 km around 38.4058 N, 110.7919 W, zoom 12-17, 699 tiles, ~10 MB,
~4 minutes (sequential, 0.3 s between requests, identifying User-Agent, hard
`--max-tiles 2000` cap). Re-running resumes; existing tiles are skipped.
Commit `tiles/mdrs/` (10 MB is fine for git). Keep the attribution
"(c) OpenStreetMap contributors" (the widget shows it from `metadata.json`).

Where the GUI looks for tiles: `$URC_TILE_DIR`, else `<package>/tiles/mdrs`
(works from a checkout or `colcon build --symlink-install`), else pass
`OfflineMapWidget(tile_dir=...)`.

## Widget usage

```python
from urc_gui_phase2.map_widget import OfflineMapWidget
w = OfflineMapWidget()                 # needs a QApplication to exist
w.set_waypoints(mission.waypoints)     # mission_model.MissionModel
w.set_local_frame(frame)               # coordinate_convert.LocalFrame
w.set_rover_enu(east_m, north_m)       # or set_rover_position(lat, lon)
w.selectionChanged.connect(lambda wp_id: ...)   # str, or None when cleared
```

Click selection: a map click picks the nearest waypoint within 20 screen
pixels (pixel distance in Mercator space at the current zoom), else clears.
Per-marker click events are not used (unreliable in pyqtlet2).

## Proving there are no network calls

Three independent layers:

1. **Nothing asks for the network.** Tile URLs are `file:///.../{z}/{x}/{y}.png`.
   Leaflet, Leaflet.draw and qwebchannel.js are the copies bundled inside
   pyqtlet2 (`file://` / `qrc://`), not CDN links (checked: no `http` in
   pyqtlet2's `map.html`/`custom.js`).
2. **The engine refuses anything non-local.** `OfflineOnlyInterceptor`
   allow-lists `file, qrc, data, blob, about` and blocks all else, recording it
   in `widget.blocked_requests`. Tests inject a real `http://` image request and
   assert it is blocked.
3. **Physical demo.** Cut the network (disable the VM adapter, or
   `nmcli networking off`), then:

   ```bash
   python3 tools/verify_offline.py --show
   ```

   It prints whether the network is really unreachable, the number of tiles
   loaded (all `file://`), and every blocked request (must be 0). It also sets
   Chromium's DNS to fail for all hosts. Exit code 0 only on PASS. If the network
   is up it says so and marks the result as weaker evidence.

## Known limits

- Raster only, fixed zoom range (12-17) and area; outside it you see a
  "no offline tile" placeholder and a red banner if a marker is out of coverage.
  Zooming past 17 is disabled rather than blurry.
- OSM street style: in the desert around MDRS there is little detail, and no
  satellite imagery or elevation.
- Tiles are a snapshot; they do not update.
- The interceptor is installed on Qt's default web profile, so it applies to
  every web view in the process.
- pyqtlet2 prints `js: Uncaught TypeError: Cannot read property 'mapObject' of
  null` and `Unhandled signal: lNObject::0` to stderr; these come from its
  event wiring at startup and are harmless here.
