#!/usr/bin/env python3
"""One-time setup tool: download OpenStreetMap tiles around MDRS for offline use.

NOT part of the running GUI. Run it once on a machine with internet, then use
the resulting directory on the rover/GUI machine with the network off:

    python3 tools/tile_downloader.py --dry-run     # count tiles, download nothing
    python3 tools/tile_downloader.py               # download to tiles/mdrs

Output is the standard XYZ layout, <out>/{z}/{x}/{y}.png, plus metadata.json
(bounds, zoom range, centre, attribution) that OfflineTileSource reads.

Being a good citizen of tile.openstreetmap.org (https://operations.osmfoundation.org/policies/tiles/):
  * sequential, single connection, with a delay between requests;
  * a User-Agent that identifies this tool;
  * small area only; a hard --max-tiles cap refuses accidental bulk downloads;
  * already-downloaded tiles are skipped, so re-running is cheap and safe.
Map data (c) OpenStreetMap contributors, ODbL. Keep the attribution.
"""

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone

# Make `urc_gui_phase2` importable when run from a checkout without installing.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from urc_gui_phase2.map_geometry import bbox_around, tiles_for_bbox  # noqa: E402

# Phase 1 challenge doc: MDRS approximately 38.4058 N, 110.7919 W.
MDRS_LAT = 38.4058
MDRS_LON = -110.7919

DEFAULT_TILE_URL = 'https://tile.openstreetmap.org/{z}/{x}/{y}.png'
DEFAULT_USER_AGENT = 'urc-gui-phase2-offline-tile-setup/1.0 (student robotics project; one-time download)'
ATTRIBUTION = '&copy; OpenStreetMap contributors'
PNG_MAGIC = b'\x89PNG\r\n\x1a\n'
AVG_TILE_KB = 15  # rough rural-area average, only used for the dry-run size estimate
MAX_RETRIES = 3


class DownloadError(Exception):
    pass


def default_out_dir() -> str:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(root, 'tiles', 'mdrs')


def plan_tiles(bbox, min_zoom, max_zoom):
    """[(z, x, y), ...] for every tile intersecting bbox, coarsest zoom first."""
    south, west, north, east = bbox
    return [(z, x, y) for z in range(min_zoom, max_zoom + 1)
            for x, y in tiles_for_bbox(south, west, north, east, z)]


def fetch(url: str, user_agent: str, timeout: float) -> bytes:
    """GET with retry/backoff on transient errors. Raises DownloadError."""
    request = urllib.request.Request(url, headers={'User-Agent': user_agent})
    for attempt in range(1, MAX_RETRIES + 1):
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                data = response.read()
            if not data.startswith(PNG_MAGIC):
                raise DownloadError(f'{url} did not return a PNG (got {data[:16]!r})')
            return data
        except urllib.error.HTTPError as exc:
            # 403/429 mean the server is telling us to stop: retrying makes it worse.
            if exc.code in (403, 429):
                raise DownloadError(f'{url}: HTTP {exc.code}; the tile server is refusing '
                                    f'requests. Stop, wait, and read its usage policy.') from exc
            if exc.code < 500 or attempt == MAX_RETRIES:
                raise DownloadError(f'{url}: HTTP {exc.code}') from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            if attempt == MAX_RETRIES:
                raise DownloadError(f'{url}: {exc}') from exc
        time.sleep(2.0 * attempt)
    raise DownloadError(f'{url}: gave up')  # unreachable; keeps type-checkers happy


def write_atomic(path: str, data: bytes) -> None:
    """Write via a temp file + rename so an interrupted run never leaves a half PNG."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + '.part'
    with open(tmp, 'wb') as handle:
        handle.write(data)
    os.replace(tmp, path)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--lat', type=float, default=MDRS_LAT, help='centre latitude (default: MDRS)')
    p.add_argument('--lon', type=float, default=MDRS_LON, help='centre longitude (default: MDRS)')
    p.add_argument('--radius-km', type=float, default=2.5,
                   help='half-side of the square area, km (default 2.5 -> 5x5 km)')
    p.add_argument('--min-zoom', type=int, default=12)
    p.add_argument('--max-zoom', type=int, default=17)
    p.add_argument('--out', default=default_out_dir(), help='output directory (default: tiles/mdrs)')
    p.add_argument('--tile-url', default=DEFAULT_TILE_URL)
    p.add_argument('--user-agent', default=DEFAULT_USER_AGENT)
    p.add_argument('--delay', type=float, default=0.3, help='seconds between requests (default 0.3)')
    p.add_argument('--timeout', type=float, default=20.0)
    p.add_argument('--max-tiles', type=int, default=2000,
                   help='refuse to run if the plan exceeds this many tiles (default 2000)')
    p.add_argument('--force', action='store_true', help='re-download tiles that already exist')
    p.add_argument('--dry-run', action='store_true', help='print the plan and exit')
    args = p.parse_args(argv)
    if not (0 <= args.min_zoom <= args.max_zoom <= 19):
        p.error('need 0 <= min-zoom <= max-zoom <= 19')
    if args.radius_km <= 0:
        p.error('--radius-km must be positive')
    if args.delay < 0.1 and 'openstreetmap.org' in args.tile_url:
        p.error('--delay below 0.1 s is not acceptable against tile.openstreetmap.org')
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    bbox = bbox_around(args.lat, args.lon, args.radius_km * 1000.0)
    plan = plan_tiles(bbox, args.min_zoom, args.max_zoom)

    print(f'Centre {args.lat:.4f}, {args.lon:.4f}; {2 * args.radius_km:g} x {2 * args.radius_km:g} km; '
          f'bbox S{bbox[0]:.5f} W{bbox[1]:.5f} N{bbox[2]:.5f} E{bbox[3]:.5f}')
    for z in range(args.min_zoom, args.max_zoom + 1):
        print(f'  zoom {z:2d}: {sum(1 for t in plan if t[0] == z):5d} tiles')
    print(f'  total  : {len(plan):5d} tiles, roughly {len(plan) * AVG_TILE_KB / 1024:.1f} MB '
          f'(~{len(plan) * args.delay / 60:.1f} min at {args.delay:g} s/tile)')

    if len(plan) > args.max_tiles:
        print(f'Refusing: {len(plan)} tiles exceeds --max-tiles {args.max_tiles}. Shrink the area '
              f'or zoom range; do not bulk-download from the public OSM servers.', file=sys.stderr)
        return 2
    if args.dry_run:
        return 0

    fetched = skipped = 0
    try:
        for i, (z, x, y) in enumerate(plan, 1):
            path = os.path.join(args.out, str(z), str(x), f'{y}.png')
            if os.path.isfile(path) and not args.force:
                skipped += 1
                continue
            url = args.tile_url.format(z=z, x=x, y=y)
            write_atomic(path, fetch(url, args.user_agent, args.timeout))
            fetched += 1
            if fetched % 25 == 0 or i == len(plan):
                print(f'  {i}/{len(plan)} tiles processed ({fetched} downloaded, {skipped} already present)')
            time.sleep(args.delay)
    except DownloadError as exc:
        print(f'ERROR: {exc}\nStopped early; re-run to resume (existing tiles are kept).',
              file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('\nInterrupted; re-run to resume.', file=sys.stderr)
        return 130

    metadata = {
        'name': 'MDRS',
        'center': [args.lat, args.lon],
        'bounds': list(bbox),  # south, west, north, east
        'min_zoom': args.min_zoom,
        'max_zoom': args.max_zoom,
        'tile_count': len(plan),
        'attribution': ATTRIBUTION,
        'source': args.tile_url,
        'downloaded_utc': datetime.now(timezone.utc).isoformat(timespec='seconds'),
    }
    with open(os.path.join(args.out, 'metadata.json'), 'w', encoding='utf-8') as handle:
        json.dump(metadata, handle, indent=2)
        handle.write('\n')
    print(f'Done: {fetched} downloaded, {skipped} already present. Tiles in {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
