"""Web Mercator / slippy-map tile math. Pure Python: no Qt, no ROS, no network.

Shared by the one-time tile downloader, OfflineTileSource, and the map
widget's click-to-select logic, so all three agree on what a tile is.

Tile scheme: standard XYZ ("slippy map"), origin at the top-left (north-west)
corner, y increasing southwards. This is what OpenStreetMap serves and what
Leaflet's L.tileLayer expects by default (no TMS y-flip).
"""

import math
from typing import Iterable, Iterator, Optional, Sequence, Tuple

TILE_SIZE_PX = 256
EARTH_RADIUS_M = 6378137.0  # WGS84 semi-major axis, the sphere Web Mercator uses
# Web Mercator is undefined at the poles; this is the standard clamp.
MAX_MERCATOR_LAT_DEG = 85.0511287798066
METERS_PER_DEG_LAT = 111_320.0


def _mercator_pixels(lat_deg: float, lon_deg: float, zoom: float) -> Tuple[float, float]:
    """World pixel coordinates (x east, y south) at a zoom, origin at NW corner."""
    lat = max(-MAX_MERCATOR_LAT_DEG, min(MAX_MERCATOR_LAT_DEG, lat_deg))
    scale = TILE_SIZE_PX * 2.0 ** zoom
    x = (lon_deg + 180.0) / 360.0 * scale
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * scale
    return x, y


def latlon_to_tile(lat_deg: float, lon_deg: float, zoom: int) -> Tuple[int, int]:
    """Tile (x, y) containing a WGS84 point at the given zoom."""
    px, py = _mercator_pixels(lat_deg, lon_deg, zoom)
    n = 2 ** zoom
    # A point exactly on the antimeridian / clamp edge would give index n.
    return (min(n - 1, max(0, int(px // TILE_SIZE_PX))),
            min(n - 1, max(0, int(py // TILE_SIZE_PX))))


def tile_bounds(x: int, y: int, zoom: int) -> Tuple[float, float, float, float]:
    """(south, west, north, east) in degrees of a tile."""
    n = 2 ** zoom

    def lat_of(ty: int) -> float:
        return math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * ty / n))))

    return lat_of(y + 1), x / n * 360.0 - 180.0, lat_of(y), (x + 1) / n * 360.0 - 180.0


def bbox_around(lat_deg: float, lon_deg: float, radius_m: float) -> Tuple[float, float, float, float]:
    """(south, west, north, east) of a square of half-side radius_m around a point.

    Uses local degree lengths, accurate to well under a percent for the
    few-kilometre boxes this is used for.
    """
    dlat = radius_m / METERS_PER_DEG_LAT
    dlon = radius_m / (METERS_PER_DEG_LAT * math.cos(math.radians(lat_deg)))
    return lat_deg - dlat, lon_deg - dlon, lat_deg + dlat, lon_deg + dlon


def tiles_for_bbox(south: float, west: float, north: float, east: float,
                   zoom: int) -> Iterator[Tuple[int, int]]:
    """Every (x, y) tile at `zoom` that intersects the bounding box."""
    x0, y0 = latlon_to_tile(north, west, zoom)  # NW corner -> smallest x and y
    x1, y1 = latlon_to_tile(south, east, zoom)
    for x in range(x0, x1 + 1):
        for y in range(y0, y1 + 1):
            yield x, y


def meters_per_pixel(lat_deg: float, zoom: float) -> float:
    """Ground resolution of a 256 px tile pyramid at this latitude and zoom."""
    return (2.0 * math.pi * EARTH_RADIUS_M * math.cos(math.radians(lat_deg))
            / (TILE_SIZE_PX * 2.0 ** zoom))


def pixel_distance(lat1: float, lon1: float, lat2: float, lon2: float, zoom: float) -> float:
    """On-screen distance in pixels between two points at a map zoom.

    Measured in Mercator space, so it is exactly what the operator sees on
    the map at any latitude.
    """
    x1, y1 = _mercator_pixels(lat1, lon1, zoom)
    x2, y2 = _mercator_pixels(lat2, lon2, zoom)
    return math.hypot(x2 - x1, y2 - y1)


def pick_nearest(click_lat: float, click_lon: float,
                 candidates: Iterable[Tuple[str, float, float]],
                 zoom: float, threshold_px: float) -> Optional[str]:
    """Key of the candidate nearest the click, if within threshold_px on screen.

    candidates: (key, lat, lon) triples. On an exact tie the earlier one wins.
    The threshold is in pixels, not metres, so the click target stays the same
    size on screen at every zoom. Returns None when nothing is close enough.
    """
    best_key: Optional[str] = None
    best_px = math.inf
    for key, lat, lon in candidates:
        px = pixel_distance(click_lat, click_lon, lat, lon, zoom)
        if px <= threshold_px and px < best_px:
            best_key, best_px = key, px
    return best_key


def bounds_contain(bounds: Sequence[float], lat_deg: float, lon_deg: float) -> bool:
    """True if (lat, lon) lies inside (south, west, north, east)."""
    south, west, north, east = bounds
    return south <= lat_deg <= north and west <= lon_deg <= east
