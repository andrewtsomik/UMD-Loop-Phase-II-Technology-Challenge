# Offline map (S1: MDRS area, no live network)

## One-time setup (needs internet, run once)

Tiles are rendered **locally** from OpenStreetMap data. We do not download tiles from
`tile.openstreetmap.org`: it IP-blocks scripted bulk downloads (and answers with a block
page that looks like a valid PNG, HTTP 200).

```bash
sudo apt-get install -y osmium-tool python3-mapnik fonts-dejavu-core   # once, ~150-300 MB
cd ~/ros2_ws/src/urc_gui_phase2
tools/build_offline_tiles.sh                  # ~1-2 min -> tiles/mdrs/{z}/{x}/{y}.png + metadata.json
```

What it does:

1. Asks the Overpass API for all OSM data in one bounding box around MDRS (a single
   request, ~13 MB; retried with backoff if the server is busy).
2. `osmium export` converts it to GeoJSON (kept in `build/`, git-ignored).
3. `tools/render_tiles.py` renders zoom 12-17 (699 tiles, ~3 MB) with Mapnik using a
   small hand-written style (roads and tracks, water, buildings, land use, cliffs, and
   place, road and building labels). No PostGIS/osm2pgsql/openstreetmap-carto needed.
4. `tools/tile_downloader.py --verify` checks the result.

Options: `BBOX="S,W,N,E"` and `OUT=dir` override the data area and output; `REFETCH=1`
re-queries Overpass instead of reusing `build/mdrs.osm`. Render only:
`python3 tools/render_tiles.py --geojson build/mdrs.geojson [--dry-run]`.
Commit `tiles/mdrs/`. Re-running overwrites the tiles.

Verify any tile folder at any time (no network, no re-render):

```bash
python3 tools/tile_downloader.py --verify [--out tiles/mdrs]
```

Hard failure (exit 1): a file that is not a PNG, or one matching the known OSM "access
blocked" placeholder hash. Byte-identical tile clusters are only a warning, since blank
desert tiles are legitimately identical (about 55% of ours are).
`tools/tile_downloader.py` (without `--verify`) still exists to fetch from a tile server
you are allowed to use via `--tile-url`; it refuses blocked responses instead of saving them.

Where the GUI looks for tiles: `$URC_TILE_DIR`, else `<package>/tiles/mdrs`
(works from a checkout or `colcon build --symlink-install`), else pass
`OfflineMapWidget(tile_dir=...)`.

## Attribution and licence (required)

The map data is (c) OpenStreetMap contributors and available under the
[Open Database Licence (ODbL) 1.0](https://www.openstreetmap.org/copyright). The rendered
tiles are a produced work from that data, so:

- Keep the credit "(c) OpenStreetMap contributors" visible wherever the map is shown. The
  widget reads it from `metadata.json` (`attribution`); do not remove it.
- Credit it in any report, slides or README that shows the map, and link
  https://www.openstreetmap.org/copyright.
- If you redistribute the underlying data (`build/mdrs.osm` / `.geojson`), it stays under
  ODbL: keep the attribution and licence note with it. (`build/` is not committed.)
- Overpass is a shared free service: one query per build, identifying User-Agent, no loops.

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
- Our own simple style, not the OSM website look: no terrain shading, satellite imagery or
  elevation, and around MDRS much of the map is blank desert. Labels are placed per tile, so
  one can be cut or repeated at a tile edge.
- Tiles are a snapshot; they do not update.
- The interceptor is installed on Qt's default web profile, so it applies to
  every web view in the process.
- pyqtlet2 prints `js: Uncaught TypeError: Cannot read property 'mapObject' of
  null` and `Unhandled signal: lNObject::0` to stderr; these come from its
  event wiring at startup and are harmless here.
