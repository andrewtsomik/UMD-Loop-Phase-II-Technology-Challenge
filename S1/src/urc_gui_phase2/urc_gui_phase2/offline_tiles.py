"""Offline tile source and the no-network policy. Pure Python: no Qt, no ROS.

Tiles live on disk as a standard XYZ pyramid, <root>/{z}/{x}/{y}.png, written
once by tools/tile_downloader.py on a networked machine. This module reads
that directory and hands the map widget a file:// URL template. It never
opens a socket.

The no-network rule is also expressed here, as a pure function
(is_local_url), so the policy can be unit-tested without Qt. The Qt request
interceptor in map_widget.py enforces it in the browser engine.
"""

import json
import os
import re
from typing import Optional, Tuple
from urllib.parse import quote, urlsplit

from urc_gui_phase2.map_geometry import bounds_contain

METADATA_FILE = 'metadata.json'
ENV_TILE_DIR = 'URC_TILE_DIR'

# Schemes that never leave the machine. Everything else (http, https, ftp,
# ws, wss, ...) is refused, as is an unknown scheme: the rule is an allow-list.
LOCAL_SCHEMES = frozenset({'file', 'qrc', 'data', 'blob', 'about'})

_DIGITS = re.compile(r'^\d+$')


class TileSourceError(Exception):
    """The tile directory is missing, empty or unreadable."""


def is_local_url(url: str) -> bool:
    """True if loading `url` cannot cause network traffic.

    file:// counts as local only with an empty host: file://server/share/x is
    a UNC path on Windows and must not be treated as a local file.
    """
    parts = urlsplit(url)
    scheme = parts.scheme.lower()
    if scheme not in LOCAL_SCHEMES:
        return False
    if scheme == 'file' and parts.netloc not in ('', 'localhost'):
        return False
    return True


def default_tile_dir() -> str:
    """$URC_TILE_DIR if set, else <package repo>/tiles/mdrs.

    The fallback resolves relative to this source file, so it works from a
    checkout or a `colcon build --symlink-install` tree. For a copied install,
    set URC_TILE_DIR or pass the directory to the widget explicitly.
    """
    env = os.environ.get(ENV_TILE_DIR)
    if env:
        return env
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(here), 'tiles', 'mdrs')


class OfflineTileSource:
    """A validated on-disk XYZ tile pyramid.

    Coverage (zoom range, bounds, centre) comes from metadata.json, which the
    downloader writes. If it is absent the zoom range is inferred from the
    directory names and no bounds are known.
    """

    def __init__(self, root_dir: str):
        self.root_dir = os.path.abspath(root_dir)
        if not os.path.isdir(self.root_dir):
            raise TileSourceError(
                f'tile directory {self.root_dir} does not exist; run '
                f'tools/tile_downloader.py once on a networked machine')

        meta = self._read_metadata()
        zooms = sorted(int(d) for d in os.listdir(self.root_dir)
                       if _DIGITS.match(d) and os.path.isdir(os.path.join(self.root_dir, d)))
        if not zooms:
            raise TileSourceError(
                f'{self.root_dir} contains no <z>/<x>/<y>.png tiles; run '
                f'tools/tile_downloader.py')

        self.min_zoom: int = int(meta.get('min_zoom', zooms[0]))
        self.max_zoom: int = int(meta.get('max_zoom', zooms[-1]))
        if self.min_zoom > self.max_zoom:
            raise TileSourceError(f'metadata zoom range {self.min_zoom}..{self.max_zoom} is inverted')
        bounds = meta.get('bounds')
        self.bounds: Optional[Tuple[float, float, float, float]] = (
            tuple(float(v) for v in bounds) if bounds else None)  # (south, west, north, east)
        center = meta.get('center')
        self.center: Optional[Tuple[float, float]] = (
            (float(center[0]), float(center[1])) if center else None)
        self.attribution: str = meta.get('attribution', '')
        self.name: str = meta.get('name', os.path.basename(self.root_dir))
        self.tile_count: Optional[int] = meta.get('tile_count')

    def _read_metadata(self) -> dict:
        path = os.path.join(self.root_dir, METADATA_FILE)
        if not os.path.isfile(path):
            return {}
        try:
            with open(path, encoding='utf-8') as handle:
                data = json.load(handle)
        except (OSError, ValueError) as exc:
            raise TileSourceError(f'cannot read {path}: {exc}') from exc
        if not isinstance(data, dict):
            raise TileSourceError(f'{path} must contain a JSON object')
        return data

    @property
    def url_template(self) -> str:
        """file:// URL with Leaflet's {z}/{x}/{y} placeholders.

        The directory part is percent-encoded (spaces, quotes, ...) so it is
        safe inside the JavaScript string pyqtlet2 builds; the placeholders
        are appended afterwards so they survive.
        """
        return 'file://' + quote(self.root_dir, safe='/') + '/{z}/{x}/{y}.png'

    def tile_path(self, z: int, x: int, y: int) -> Optional[str]:
        """Absolute path of a tile if it exists on disk, else None."""
        path = os.path.join(self.root_dir, str(z), str(x), f'{y}.png')
        return path if os.path.isfile(path) else None

    def covers(self, lat_deg: float, lon_deg: float) -> bool:
        """Whether a point is inside the downloaded area (True if bounds unknown)."""
        return self.bounds is None or bounds_contain(self.bounds, lat_deg, lon_deg)

    def describe(self) -> str:
        count = f', {self.tile_count} tiles' if self.tile_count else ''
        return f'{self.name}: zoom {self.min_zoom}-{self.max_zoom}{count}'
