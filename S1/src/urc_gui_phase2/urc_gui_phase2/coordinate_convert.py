"""WGS84 <-> UTM <-> local ENU conversion, anchored at the rover spawn point.

Pure Python plus pyproj: no ROS, no Qt.

Pipeline (both directions):

    WGS84 lat/lon --pyproj--> UTM (E, N) --subtract origin's UTM--> local (east, north)
    local (east, north) --add origin's UTM--> UTM (E, N) --pyproj--> WGS84 lat/lon

The UTM projection itself is done entirely by pyproj (PROJ). Nothing here
re-implements projection math; the only arithmetic is the subtraction and
addition of the origin's UTM easting/northing, and up = alt - origin_alt.

Frame convention (REP 103): right-handed ENU, x = east, y = north, z = up,
metres, origin at the spawn point.

UTM zone handling
-----------------
The zone (and hemisphere) is chosen ONCE, from the origin, and every
conversion in that frame is forced into it, even for points that would
nominally fall in a neighbouring zone. Reasons:

* UTM zones are separate projections. Each has its own easting origin
  (500 000 m at its own central meridian), so the same point has unrelated
  (E, N) values in two zones. If each point used its own nominal zone,
  subtracting the origin's (E, N) would compare numbers from different
  coordinate systems, and the local frame would jump by hundreds of km at the
  zone line. REP 103 wants one continuous local frame.
* Forcing a zone is not an error: the transverse-Mercator projection is
  defined for any longitude, and zone limits are only conventions chosen to
  keep distortion small. Using a zone slightly outside its nominal 6-degree
  band just accepts slightly more distortion. Over a mission site a few km
  across, next to an origin in that zone, the extra distortion is well below
  a GNSS receiver's noise.
* The frozen zone makes the frame deterministic: the same (origin, point)
  always yields the same ENU, regardless of where other waypoints are.

What the frame does NOT do (state these to judges):

* Y axis is UTM grid north, not true north. They differ by the meridian
  convergence at the origin (grid_convergence_deg), which is 0 on the zone's
  central meridian and grows to about 2 degrees at 45 degrees latitude at the
  zone edge. Anything mixing this frame with a true-north heading (compass,
  IMU yaw) must correct for it.
* Distances carry the UTM scale factor: 0.9996 on the central meridian,
  rising to about 1.0003 at the zone edge at 45 degrees latitude and about
  1.001 at the zone edge on the equator. ENU distances therefore differ from
  true ground distance by roughly -0.04 to +0.1 percent (0.4 to 1 m per km).
  A test measures this against a geodesic.
* Altitude is passed through (up = alt - origin_alt) with no geoid model.
  Use one altitude reference (e.g. all ellipsoidal) consistently.
* Points too far from the origin are rejected (max_range_m), because far from
  the central meridian the distortion grows, and because a valid-looking but
  wrong coordinate (a flipped E/W sign, say) would otherwise quietly become a
  waypoint thousands of km away.

Coordinate text input: decimal degrees (DD), degrees-decimal-minutes (DDM) and
degrees-minutes-seconds (DMS) are all accepted by parse_latitude() and
parse_longitude(); see those for the grammar. Malformed input raises
CoordinateFormatError, and out-of-range values raise InvalidCoordinateError
(the same error mission_model uses).

Thread safety: a LocalFrame is immutable after construction, but pyproj
Transformers should not be assumed thread-safe. Use a frame from one thread.
"""

import math
import re
from typing import Any, NamedTuple, Optional, Tuple

from pyproj import CRS, Proj, Transformer
from pyproj.exceptions import ProjError

from urc_gui_phase2.mission_model import InvalidCoordinateError, validate_coordinate

UTM_MIN_LAT_DEG = -80.0   # UTM (as opposed to UPS) is defined for 80 S .. 84 N
UTM_MAX_LAT_DEG = 84.0
DEFAULT_MAX_RANGE_M = 100_000.0


class CoordinateFormatError(InvalidCoordinateError):
    """Coordinate text is not valid DD, DDM or DMS."""


class FrameRangeError(InvalidCoordinateError):
    """A coordinate is valid but too far from the frame's origin to convert safely."""


class ENU(NamedTuple):
    east_m: float
    north_m: float
    up_m: float


class Geodetic(NamedTuple):
    lat_deg: float
    lon_deg: float
    alt_m: float


# ---- UTM zone ------------------------------------------------------------

def utm_zone_for_longitude(lon_deg: float) -> int:
    """Nominal UTM zone number (1..60) for a longitude.

    Zone n covers [-180 + 6(n-1), -180 + 6n). The Norway/Svalbard irregular
    zones are ignored on purpose: the zone here only selects a projection
    for the local frame, so any consistent choice is valid.
    """
    return min(int((lon_deg + 180.0) // 6.0) + 1, 60)


# ---- text parsing --------------------------------------------------------

_SYMBOLS = str.maketrans({c: ' ' for c in '°º˚′’\'″”"'})
_NUMBER = re.compile(r'^\d+(?:\.\d+)?$')


def _fail(text: Any, why: str) -> CoordinateFormatError:
    return CoordinateFormatError(f'cannot parse coordinate {text!r}: {why}')


def _parse_angle(value: Any, hemispheres: str) -> float:
    """Parse DD / DDM / DMS into signed decimal degrees (range not yet checked)."""
    if isinstance(value, bool):
        raise _fail(value, 'expected a number or text, not a bool')
    if isinstance(value, (int, float)):
        return float(value)
    if not isinstance(value, str):
        raise _fail(value, f'expected a number or text, got {type(value).__name__}')

    # Degree/minute/second marks become spaces; hemisphere letters get their
    # own token so "38.9N" and "38.9 N" parse the same way.
    text = re.sub(r'([NSEW])', r' \1 ', value.upper().translate(_SYMBOLS))
    tokens = text.split()
    if not tokens:
        raise _fail(value, 'empty input')

    letter_positions = [i for i, t in enumerate(tokens) if t.isalpha()]
    hemisphere = None
    if letter_positions:
        if len(letter_positions) > 1:
            raise _fail(value, 'more than one letter (only one N/S/E/W hemisphere is allowed)')
        i = letter_positions[0]
        if i not in (0, len(tokens) - 1):
            raise _fail(value, 'hemisphere letter must come first or last')
        hemisphere = tokens.pop(i)
        if hemisphere not in hemispheres:
            raise _fail(value, f'hemisphere {hemisphere!r} does not fit here '
                               f'(expected {" or ".join(hemispheres)})')

    sign = ''
    if tokens and tokens[0] in ('+', '-'):
        sign = tokens.pop(0)
    elif tokens and tokens[0][0] in '+-':
        sign, tokens[0] = tokens[0][0], tokens[0][1:]
    if sign and hemisphere:
        raise _fail(value, 'give either a +/- sign or a hemisphere letter, not both')

    if not 1 <= len(tokens) <= 3:
        raise _fail(value, f'expected 1 (DD), 2 (DDM) or 3 (DMS) numbers, got {len(tokens)}')
    for token in tokens:
        if not _NUMBER.match(token):
            raise _fail(value, f'{token!r} is not a plain non-negative number')

    # Only the last field may have a fractional part.
    for token in tokens[:-1]:
        if '.' in token:
            raise _fail(value, f'only the last field may be fractional, got {token!r}')
    numbers = [float(t) for t in tokens]
    for label, number in zip(('minutes', 'seconds'), numbers[1:]):
        if number >= 60.0:
            raise _fail(value, f'{label} must be less than 60, got {number:g}')

    degrees = numbers[0]
    if len(numbers) >= 2:
        degrees += numbers[1] / 60.0
    if len(numbers) == 3:
        degrees += numbers[2] / 3600.0

    negative = sign == '-' or hemisphere in ('S', 'W')
    return -degrees if negative else degrees


def parse_latitude(value: Any) -> float:
    """Parse a latitude to signed decimal degrees, validated to [-90, 90].

    Accepts a number, or text in any of (hemisphere N/S may replace the sign,
    at the start or end; degree/minute/second marks are optional):

        DD   38.9869   -38.9869   38.9869 N   38.9869°S
        DDM  38 59.214   38° 59.214' N   -38 59.214
        DDM/DMS  38 59 12.84   38° 59' 12.84" S

    Rules: the number of fields picks the format (1 = DD, 2 = DDM, 3 = DMS);
    only the last field may have a decimal part; minutes and seconds must be
    below 60; a sign and a hemisphere letter together are rejected; a
    hemisphere letter must suit the axis (N/S here). Raises
    CoordinateFormatError for malformed text and InvalidCoordinateError for a
    result outside the valid range.
    """
    return validate_coordinate(_parse_angle(value, 'NS'), 0.0)[0]


def parse_longitude(value: Any) -> float:
    """Parse a longitude to signed decimal degrees, validated to [-180, 180].

    Same grammar as parse_latitude, with hemisphere letters E/W.
    """
    return validate_coordinate(0.0, _parse_angle(value, 'EW'))[1]


def parse_lat_lon(lat_value: Any, lon_value: Any) -> Tuple[float, float]:
    """Parse a latitude and a longitude (each in any format) to decimal degrees."""
    return parse_latitude(lat_value), parse_longitude(lon_value)


FORMAT_DD = 'DD'
FORMAT_DDM = 'DDM'
FORMAT_DMS = 'DMS'
COORDINATE_FORMATS = (FORMAT_DD, FORMAT_DDM, FORMAT_DMS)

_DD_DECIMALS = 6      # ~0.1 m
_DDM_DECIMALS = 4     # minutes: ~0.2 m
_DMS_DECIMALS = 2     # seconds: ~0.3 m


def _format_angle(value: float, hemispheres: str, fmt: str) -> str:
    if fmt not in COORDINATE_FORMATS:
        raise ValueError(f'unknown coordinate format {fmt!r}; use one of {COORDINATE_FORMATS}')
    hemisphere = hemispheres[1] if value < 0 else hemispheres[0]
    magnitude = abs(float(value))
    if fmt == FORMAT_DD:
        return f'{magnitude:.{_DD_DECIMALS}f}\N{DEGREE SIGN} {hemisphere}'
    # Round in integer units of the smallest field so carries (59.99999' -> 60')
    # propagate correctly instead of printing a field of 60.
    if fmt == FORMAT_DDM:
        scale = 10 ** _DDM_DECIMALS
        units = round(magnitude * 60 * scale)
        degrees, rest = divmod(units, 60 * scale)
        return (f"{degrees}\N{DEGREE SIGN} {rest / scale:.{_DDM_DECIMALS}f}' {hemisphere}")
    scale = 10 ** _DMS_DECIMALS
    units = round(magnitude * 3600 * scale)
    degrees, rest = divmod(units, 3600 * scale)
    minutes, sec_units = divmod(rest, 60 * scale)
    return (f"{degrees}\N{DEGREE SIGN} {minutes}' "
            f'{sec_units / scale:.{_DMS_DECIMALS}f}" {hemisphere}')


def format_latitude(lat_deg: float, fmt: str = FORMAT_DD) -> str:
    """Format a latitude as DD, DDM or DMS text that parse_latitude() reads back."""
    return _format_angle(validate_coordinate(lat_deg, 0.0)[0], 'NS', fmt)


def format_longitude(lon_deg: float, fmt: str = FORMAT_DD) -> str:
    """Format a longitude as DD, DDM or DMS text that parse_longitude() reads back."""
    return _format_angle(validate_coordinate(0.0, lon_deg)[1], 'EW', fmt)


# ---- the local frame -----------------------------------------------------

class LocalFrame:
    """Local ENU frame anchored at a WGS84 origin (the rover spawn point)."""

    def __init__(self, origin_lat_deg: float, origin_lon_deg: float,
                 origin_alt_m: float = 0.0, *, max_range_m: float = DEFAULT_MAX_RANGE_M):
        lat, lon = validate_coordinate(origin_lat_deg, origin_lon_deg)
        if not UTM_MIN_LAT_DEG <= lat <= UTM_MAX_LAT_DEG:
            raise InvalidCoordinateError(
                f'origin latitude {lat} is outside the UTM range '
                f'[{UTM_MIN_LAT_DEG:g}, {UTM_MAX_LAT_DEG:g}]')
        if isinstance(origin_alt_m, bool) or not isinstance(origin_alt_m, (int, float)) \
                or not math.isfinite(origin_alt_m):
            raise InvalidCoordinateError(f'origin altitude must be a finite number, '
                                         f'got {origin_alt_m!r}')
        if not (isinstance(max_range_m, (int, float)) and math.isfinite(max_range_m)
                and max_range_m > 0):
            raise ValueError(f'max_range_m must be a positive finite number, got {max_range_m!r}')

        self.origin_lat_deg = lat
        self.origin_lon_deg = lon
        self.origin_alt_m = float(origin_alt_m)
        self.max_range_m = float(max_range_m)

        # Zone and hemisphere are fixed here, once, from the origin.
        self.utm_zone = utm_zone_for_longitude(lon)
        self.northern_hemisphere = lat >= 0.0
        epsg = (32600 if self.northern_hemisphere else 32700) + self.utm_zone
        self._crs = CRS.from_epsg(epsg)  # WGS84 / UTM zone N
        # always_xy=True: EPSG:4326 officially orders (lat, lon); we always
        # pass (lon, lat) so the order can't silently flip.
        self._to_utm = Transformer.from_crs('EPSG:4326', self._crs, always_xy=True)
        self._from_utm = Transformer.from_crs(self._crs, 'EPSG:4326', always_xy=True)

        self.origin_easting_m, self.origin_northing_m = self._project(lat, lon)

        factors = Proj(self._crs).get_factors(lon, lat)
        self._grid_convergence_deg = factors.meridian_convergence
        self._scale_factor = factors.meridional_scale

    @property
    def epsg(self) -> int:
        return self._crs.to_epsg()

    @property
    def grid_convergence_deg(self) -> float:
        """Angle at the origin from true north to grid north (this frame's +y).

        Positive when grid north lies east (clockwise) of true north, as
        PROJ reports it. A true-north bearing B corresponds to grid bearing
        B - grid_convergence_deg.
        """
        return self._grid_convergence_deg

    @property
    def scale_factor(self) -> float:
        """UTM point scale factor at the origin (grid distance / ground distance)."""
        return self._scale_factor

    # -- UTM level -----------------------------------------------------------

    def to_utm(self, lat_deg: float, lon_deg: float) -> Tuple[float, float]:
        """WGS84 -> (easting, northing) in this frame's forced zone.

        Applies no range limit; to_enu does.
        """
        lat, lon = validate_coordinate(lat_deg, lon_deg)
        return self._project(lat, lon)

    def from_utm(self, easting_m: float, northing_m: float) -> Tuple[float, float]:
        """(easting, northing) in this frame's zone -> (lat_deg, lon_deg)."""
        for label, value in (('easting', easting_m), ('northing', northing_m)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or not math.isfinite(value):
                raise InvalidCoordinateError(f'{label} must be a finite number, got {value!r}')
        try:
            lon, lat = self._from_utm.transform(easting_m, northing_m, errcheck=True)
        except ProjError as exc:
            raise InvalidCoordinateError(f'UTM to WGS84 failed: {exc}') from exc
        return lat, lon

    # -- ENU level -------------------------------------------------------------

    def to_enu(self, lat_deg: float, lon_deg: float, alt_m: Optional[float] = None) -> ENU:
        """WGS84 -> local ENU (metres) relative to the origin.

        alt_m=None means "same height as the origin" (up = 0), which is what
        a 2D waypoint means. Raises FrameRangeError beyond max_range_m.
        """
        lat, lon = validate_coordinate(lat_deg, lon_deg)
        up = 0.0 if alt_m is None else self._finite(alt_m, 'altitude') - self.origin_alt_m
        easting, northing = self._project(lat, lon)
        east = easting - self.origin_easting_m
        north = northing - self.origin_northing_m
        distance = math.hypot(east, north)
        if distance > self.max_range_m:
            raise FrameRangeError(
                f'({lat}, {lon}) is {distance / 1000:.1f} km from the frame origin '
                f'(limit {self.max_range_m / 1000:.1f} km); check for a wrong sign or typo')
        return ENU(east, north, up)

    def to_wgs84(self, east_m: float, north_m: float, up_m: float = 0.0) -> Geodetic:
        """Local ENU (metres) -> WGS84. The inverse of to_enu."""
        east = self._finite(east_m, 'east')
        north = self._finite(north_m, 'north')
        up = self._finite(up_m, 'up')
        distance = math.hypot(east, north)
        if distance > self.max_range_m:
            raise FrameRangeError(
                f'ENU ({east}, {north}) is {distance / 1000:.1f} km from the frame origin '
                f'(limit {self.max_range_m / 1000:.1f} km)')
        lat, lon = self.from_utm(self.origin_easting_m + east, self.origin_northing_m + north)
        return Geodetic(lat, lon, self.origin_alt_m + up)

    # -- internals -------------------------------------------------------------

    def _project(self, lat: float, lon: float) -> Tuple[float, float]:
        try:
            easting, northing = self._to_utm.transform(lon, lat, errcheck=True)
        except ProjError as exc:
            raise InvalidCoordinateError(f'WGS84 to UTM failed for ({lat}, {lon}): {exc}') from exc
        return easting, northing

    @staticmethod
    def _finite(value: Any, label: str) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)) \
                or not math.isfinite(value):
            raise InvalidCoordinateError(f'{label} must be a finite number, got {value!r}')
        return float(value)
