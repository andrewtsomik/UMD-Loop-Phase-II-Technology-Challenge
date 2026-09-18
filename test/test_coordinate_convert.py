import math

from pyproj import Geod, Transformer
import pytest

from urc_gui_phase2.coordinate_convert import (
    CoordinateFormatError,
    FrameRangeError,
    LocalFrame,
    parse_lat_lon,
    parse_latitude,
    parse_longitude,
    utm_zone_for_longitude,
)
from urc_gui_phase2.mission_model import InvalidCoordinateError, MissionModel

# ---- independent oracle ---------------------------------------------------
# These helpers do NOT use pyproj. They implement the transverse-Mercator
# series from Snyder, "Map Projections - A Working Manual" (USGS PP 1395),
# eqs. 3-21, 8-9..8-13, plus a numerical integral for the meridian arc, so the
# known-answer tests below check pyproj against something other than itself.
# They live in the tests only: the module itself must not hand-roll UTM.

WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
E2 = WGS84_F * (2 - WGS84_F)
EP2 = E2 / (1 - E2)
K0 = 0.9996


def meridian_arc(phi_rad, steps=2000):
    """Distance along a meridian from the equator to latitude phi (Simpson's rule)."""
    def integrand(p):
        return WGS84_A * (1 - E2) * (1 - E2 * math.sin(p) ** 2) ** -1.5
    h = phi_rad / steps
    total = integrand(0.0) + integrand(phi_rad)
    for i in range(1, steps):
        total += (4 if i % 2 else 2) * integrand(i * h)
    return total * h / 3


def snyder_utm(lat_deg, lon_deg, central_meridian_deg):
    """Northern-hemisphere UTM (E, N) from the series; accurate to ~0.1 mm within 3 degrees."""
    phi = math.radians(lat_deg)
    t = math.tan(phi) ** 2
    c = EP2 * math.cos(phi) ** 2
    a = math.radians(lon_deg - central_meridian_deg) * math.cos(phi)
    nu = WGS84_A / math.sqrt(1 - E2 * math.sin(phi) ** 2)
    x = K0 * nu * (a + (1 - t + c) * a ** 3 / 6
                   + (5 - 18 * t + t * t + 72 * c - 58 * EP2) * a ** 5 / 120)
    y = K0 * (meridian_arc(phi) + nu * math.tan(phi) * (
        a ** 2 / 2 + (5 - t + 9 * c + 4 * c * c) * a ** 4 / 24
        + (61 - 58 * t + t * t + 600 * c - 330 * EP2) * a ** 6 / 720))
    return 500000.0 + x, y


GEOD = Geod(ellps='WGS84')

# Tolerances (see the report for the reasoning; measured errors are far smaller):
ROUND_TRIP_DEG = 1e-9   # ~0.1 mm on the ground; measured worst case ~4e-14 deg
ROUND_TRIP_M = 1e-6     # 1 micrometre; measured worst case ~6e-9 m
ORACLE_M = 1e-3         # 1 mm vs the independent series; measured worst case ~3e-5 m


# ---- known-answer: UTM values checked independently ------------------------

def test_equator_on_central_meridian_is_false_easting_and_zero_northing():
    # By definition of UTM: E = 500000 m on the central meridian, N = 0 at the
    # equator (zone 33's central meridian is 15 E).
    frame = LocalFrame(0.0, 15.0)
    e, n = frame.to_utm(0.0, 15.0)
    assert e == pytest.approx(500000.0, abs=1e-6)
    assert n == pytest.approx(0.0, abs=1e-6)


def test_45N_on_central_meridian_matches_published_and_computed_value():
    frame = LocalFrame(45.0, 15.0)
    e, n = frame.to_utm(45.0, 15.0)
    assert e == pytest.approx(500000.0, abs=1e-6)
    # Published: k0 * meridian arc(45 N) = 0.9996 * 4 984 944.378 m = 4 982 950.400 m.
    assert n == pytest.approx(4982950.400, abs=1e-3)
    # And computed independently by numerically integrating the meridian arc.
    assert n == pytest.approx(K0 * meridian_arc(math.radians(45.0)), abs=ORACLE_M)


def test_45S_on_central_meridian_uses_false_northing():
    frame = LocalFrame(-45.0, 15.0)
    assert frame.epsg == 32733
    e, n = frame.to_utm(-45.0, 15.0)
    assert e == pytest.approx(500000.0, abs=1e-6)
    assert n == pytest.approx(10_000_000.0 - K0 * meridian_arc(math.radians(45.0)), abs=ORACLE_M)


@pytest.mark.parametrize('lat, lon', [
    (38.9, 14.0), (52.1, 16.5), (10.0, 13.2), (60.0, 17.0), (30.0, 12.1), (45.0, 17.9),
])
def test_matches_independent_series_off_the_central_meridian(lat, lon):
    frame = LocalFrame(lat, lon)  # zone 33, central meridian 15 E
    assert frame.utm_zone == 33
    e, n = frame.to_utm(lat, lon)
    oracle_e, oracle_n = snyder_utm(lat, lon, 15.0)
    assert e == pytest.approx(oracle_e, abs=ORACLE_M)
    assert n == pytest.approx(oracle_n, abs=ORACLE_M)


def test_enu_is_utm_offset_from_the_origin():
    frame = LocalFrame(45.0, 15.0)
    east, north, up = frame.to_enu(45.001, 15.002)
    e0, n0 = frame.to_utm(45.0, 15.0)
    e1, n1 = frame.to_utm(45.001, 15.002)
    assert (east, north, up) == pytest.approx((e1 - e0, n1 - n0, 0.0), abs=1e-9)
    # ...and that matches the independent series too, not just the same pyproj call.
    oe0, on0 = snyder_utm(45.0, 15.0, 15.0)
    oe1, on1 = snyder_utm(45.001, 15.002, 15.0)
    assert east == pytest.approx(oe1 - oe0, abs=ORACLE_M)
    assert north == pytest.approx(on1 - on0, abs=ORACLE_M)


@pytest.mark.parametrize('lon, zone', [
    (-180.0, 1), (-174.0001, 1), (-174.0, 2), (0.0, 31), (5.9999, 31), (6.0, 32),
    (15.0, 33), (179.9999, 60), (180.0, 60),
])
def test_utm_zone_numbers(lon, zone):
    assert utm_zone_for_longitude(lon) == zone


# ---- ENU frame behaviour ---------------------------------------------------

def test_origin_maps_to_zero_and_axes_point_east_north_up():
    frame = LocalFrame(38.9869, -76.9426, origin_alt_m=50.0)
    assert frame.to_enu(38.9869, -76.9426, 50.0) == pytest.approx((0, 0, 0), abs=1e-9)
    east, north, up = frame.to_enu(38.9869 + 0.001, -76.9426 + 0.001, 62.5)
    assert east > 0 and north > 0            # +x east, +y north
    assert up == pytest.approx(12.5)         # z is altitude above the origin
    # Right size: compare the ENU distance with the true geodesic distance. (Comparing
    # `north` alone to 111 m would be wrong: this site is ~1.9 degrees from its central
    # meridian, so grid north is rotated ~1.2 degrees from true north -- see the
    # grid-convergence test -- and part of the eastward step shows up in `north`.)
    geodesic = GEOD.inv(-76.9426, 38.9869, -76.9426 + 0.001, 38.9869 + 0.001)[2]
    assert math.hypot(east, north) == pytest.approx(geodesic, rel=1e-3)


def test_altitude_defaults_to_origin_height():
    frame = LocalFrame(10.0, 10.0, origin_alt_m=123.0)
    assert frame.to_enu(10.0, 10.001).up_m == 0.0
    assert frame.to_wgs84(0.0, 0.0, 5.0).alt_m == pytest.approx(128.0)


@pytest.mark.parametrize('origin', [
    (38.9869, -76.9426),   # US east coast, northern hemisphere
    (-33.8688, 151.2093),  # southern hemisphere
    (0.0, 0.0),
    (64.1466, -21.9426),   # high latitude
    (45.0, 5.9999),        # right at a zone edge
])
def test_round_trip_wgs84_enu_wgs84(origin):
    lat0, lon0 = origin
    frame = LocalFrame(lat0, lon0, origin_alt_m=10.0)
    for dlat, dlon in [(0, 0), (0.01, 0.01), (-0.02, 0.005), (0.0499, -0.0499), (-0.03, -0.03)]:
        lat, lon = lat0 + dlat, lon0 + dlon
        enu = frame.to_enu(lat, lon, 25.0)
        back = frame.to_wgs84(*enu)
        assert back.lat_deg == pytest.approx(lat, abs=ROUND_TRIP_DEG)
        assert back.lon_deg == pytest.approx(lon, abs=ROUND_TRIP_DEG)
        assert back.alt_m == pytest.approx(25.0, abs=ROUND_TRIP_M)


def test_round_trip_enu_wgs84_enu():
    frame = LocalFrame(38.9869, -76.9426)
    for east, north, up in [(0, 0, 0), (1500.0, -900.0, 3.0), (-4000.0, 4000.0, -1.0)]:
        g = frame.to_wgs84(east, north, up)
        back = frame.to_enu(g.lat_deg, g.lon_deg, g.alt_m)
        assert back == pytest.approx((east, north, up), abs=ROUND_TRIP_M)


# ---- UTM zone boundary -----------------------------------------------------
# Origin at 5.9 E is in zone 31 (0..6 E); the target at 6.1 E is nominally in
# zone 32. The frame must keep using zone 31 for both.

def test_frame_forces_the_origins_zone_across_a_zone_boundary():
    frame = LocalFrame(45.0, 5.9)
    assert frame.utm_zone == 31 and frame.epsg == 32631
    assert utm_zone_for_longitude(6.1) == 32  # the target's nominal zone differs

    east, north, _ = frame.to_enu(45.0, 6.1)

    # Expected: both points projected in zone 31, computed directly with pyproj.
    zone31 = Transformer.from_crs('EPSG:4326', 'EPSG:32631', always_xy=True)
    e0, n0 = zone31.transform(5.9, 45.0)
    e1, n1 = zone31.transform(6.1, 45.0)
    assert (east, north) == pytest.approx((e1 - e0, n1 - n0), abs=1e-6)


def test_forced_zone_gives_a_continuous_frame_where_per_point_zones_would_jump():
    frame = LocalFrame(45.0, 5.9)
    just_west = frame.to_enu(45.0, 5.9999)
    just_east = frame.to_enu(45.0, 6.0001)  # 0.0002 deg apart, straddling the zone line
    step = math.hypot(just_east.east_m - just_west.east_m, just_east.north_m - just_west.north_m)
    true_gap = GEOD.inv(5.9999, 45.0, 6.0001, 45.0)[2]
    assert step == pytest.approx(true_gap, rel=1e-3)   # ~15.7 m, no jump
    assert step < 20.0

    # What the rejected alternative does: project each point in its OWN nominal
    # zone and subtract. The two easting systems are unrelated, so it "jumps".
    zone31 = Transformer.from_crs('EPSG:4326', 'EPSG:32631', always_xy=True)
    zone32 = Transformer.from_crs('EPSG:4326', 'EPSG:32632', always_xy=True)
    naive_west = zone31.transform(5.9999, 45.0)[0]
    naive_east = zone32.transform(6.0001, 45.0)[0]
    assert abs(naive_east - naive_west) > 100_000.0   # hundreds of km for a 16 m step


def test_distances_across_the_zone_boundary_agree_with_a_geodesic():
    # Also measures the planar/UTM approximation: 3 degrees from the central
    # meridian at 45 N is near the worst case for a real mission site.
    frame = LocalFrame(45.0, 5.9)
    for lon in (5.95, 6.0, 6.05, 6.1):
        east, north, _ = frame.to_enu(45.0, lon)
        geodesic = GEOD.inv(5.9, 45.0, lon, 45.0)[2]
        assert math.hypot(east, north) == pytest.approx(geodesic, rel=1e-3)


def test_round_trip_across_the_zone_boundary():
    frame = LocalFrame(45.0, 5.9)
    for lon in (5.95, 6.0, 6.0001, 6.1):
        back = frame.to_wgs84(*frame.to_enu(45.01, lon))
        assert back.lat_deg == pytest.approx(45.01, abs=ROUND_TRIP_DEG)
        assert back.lon_deg == pytest.approx(lon, abs=ROUND_TRIP_DEG)


def test_frame_across_the_antimeridian_zone_60_to_zone_1():
    frame = LocalFrame(45.0, 179.9)
    assert frame.utm_zone == 60
    assert utm_zone_for_longitude(-179.9) == 1
    east, north, _ = frame.to_enu(45.0, -179.9)   # 0.2 degrees east, across +-180
    geodesic = GEOD.inv(179.9, 45.0, -179.9, 45.0)[2]
    assert east > 0
    assert math.hypot(east, north) == pytest.approx(geodesic, rel=1e-3)
    back = frame.to_wgs84(east, north)
    assert back.lon_deg == pytest.approx(-179.9, abs=ROUND_TRIP_DEG)


def test_hemisphere_is_frozen_too_so_the_equator_is_not_a_seam():
    frame = LocalFrame(0.0005, 10.0)   # just north of the equator
    assert frame.northern_hemisphere
    below = frame.to_enu(-0.0005, 10.0)
    assert below.north_m == pytest.approx(-110.6, abs=0.5)   # negative, no 10,000 km jump
    assert frame.to_wgs84(*below).lat_deg == pytest.approx(-0.0005, abs=ROUND_TRIP_DEG)


# ---- grid convergence and scale (what the ENU frame does not hide) -----------

def test_grid_convergence_matches_a_true_north_geodesic():
    frame = LocalFrame(45.0, 5.9)
    lon2, lat2, _ = GEOD.fwd(5.9, 45.0, 0.0, 1000.0)   # 1 km due TRUE north
    east, north, _ = frame.to_enu(lat2, lon2)
    bearing_in_grid = math.degrees(math.atan2(east, north))
    assert bearing_in_grid == pytest.approx(-frame.grid_convergence_deg, abs=0.01)
    # First-order check: convergence ~ (lon - central meridian) * sin(lat) = 2.9 deg * sin(45).
    assert frame.grid_convergence_deg == pytest.approx(
        (5.9 - 3.0) * math.sin(math.radians(45.0)), rel=0.02)


def test_convergence_zero_and_scale_k0_on_the_central_meridian():
    frame = LocalFrame(45.0, 15.0)
    assert frame.grid_convergence_deg == pytest.approx(0.0, abs=1e-9)
    assert frame.scale_factor == pytest.approx(0.9996, abs=1e-9)


# ---- guards ----------------------------------------------------------------

def test_flipped_longitude_sign_is_rejected_not_silently_converted():
    frame = LocalFrame(38.9869, -76.9426)
    with pytest.raises(FrameRangeError, match='km from the frame origin'):
        frame.to_enu(38.9869, 76.9426)   # E instead of W: valid coordinate, wrong place
    with pytest.raises(FrameRangeError):
        frame.to_wgs84(5_000_000.0, 0.0)


def test_max_range_is_configurable():
    frame = LocalFrame(45.0, 15.0, max_range_m=1000.0)
    frame.to_enu(45.001, 15.0)
    with pytest.raises(FrameRangeError):
        frame.to_enu(45.02, 15.0)


@pytest.mark.parametrize('lat, lon', [
    (85.0, 0.0), (-80.5, 0.0), (91.0, 0.0), (0.0, 181.0), (float('nan'), 0.0), ('45', 10.0),
])
def test_invalid_origin_is_rejected(lat, lon):
    with pytest.raises(InvalidCoordinateError):
        LocalFrame(lat, lon)


def test_invalid_inputs_to_conversions_are_rejected():
    frame = LocalFrame(45.0, 15.0)
    with pytest.raises(InvalidCoordinateError):
        frame.to_enu(95.0, 15.0)
    with pytest.raises(InvalidCoordinateError):
        frame.to_enu(45.0, float('nan'))
    with pytest.raises(InvalidCoordinateError):
        frame.to_enu(45.0, 15.0, alt_m=float('inf'))
    with pytest.raises(InvalidCoordinateError):
        frame.to_wgs84(float('nan'), 0.0)
    with pytest.raises(InvalidCoordinateError):
        frame.to_wgs84(True, 0.0)


# ---- link to mission_model: WGS84 stored, ENU derived -----------------------

def test_stored_wgs84_is_independent_of_the_frame_anchor():
    mission = MissionModel()
    wp = mission.add('Post A', 38.9869, -76.9426)

    frame_a = LocalFrame(38.9860, -76.9430)   # one spawn point
    frame_b = LocalFrame(38.9900, -76.9400)   # the operator moves the spawn point later
    enu_a = frame_a.to_enu(*wp.coordinate)
    enu_b = frame_b.to_enu(*wp.coordinate)

    assert enu_a != pytest.approx(enu_b, abs=1.0)   # the ENU value depends on the anchor...
    assert mission.get(wp.id).coordinate == (38.9869, -76.9426)   # ...the stored waypoint doesn't
    for frame, enu in ((frame_a, enu_a), (frame_b, enu_b)):
        back = frame.to_wgs84(*enu)
        assert (back.lat_deg, back.lon_deg) == pytest.approx(wp.coordinate, abs=ROUND_TRIP_DEG)


# ---- parsing: DD / DDM / DMS ---------------------------------------------------

@pytest.mark.parametrize('text, expected', [
    ('38.9869', 38.9869),
    ('-38.9869', -38.9869),
    ('+38.9869', 38.9869),
    (38.9869, 38.9869),
    (39, 39.0),
    ('38.9869 N', 38.9869),
    ('38.9869N', 38.9869),
    ('N 38.9869', 38.9869),
    ('38.9869°S', -38.9869),
    ('  38.9869°  n  ', 38.9869),
    ('38 59.214', 38 + 59.214 / 60),                       # DDM
    ("38° 59.214' N", 38 + 59.214 / 60),
    ('38°59.214′N', 38 + 59.214 / 60),
    ('38 59.214 S', -(38 + 59.214 / 60)),
    ('-38 59.214', -(38 + 59.214 / 60)),
    ('38 59 12.84', 38 + 59 / 60 + 12.84 / 3600),          # DMS
    ('38° 59\' 12.84" N', 38 + 59 / 60 + 12.84 / 3600),
    ('38°59′12.84″N', 38 + 59 / 60 + 12.84 / 3600),
    ('S 38 59 12.84', -(38 + 59 / 60 + 12.84 / 3600)),
    ('-0 30', -0.5),                                        # negative zero degrees keeps its sign
    ('S 0 30 0', -0.5),
    ('0', 0.0),
    ('90 N', 90.0),
    ("90° 0' 0\" S", -90.0),
])
def test_parse_latitude_accepts(text, expected):
    assert parse_latitude(text) == pytest.approx(expected, abs=1e-12)


@pytest.mark.parametrize('text, expected', [
    ('-76.9426', -76.9426),
    ('76.9426 W', -76.9426),
    ('76.9426 E', 76.9426),
    ("76° 56.556' W", -(76 + 56.556 / 60)),
    ('76 56 33.36 W', -(76 + 56 / 60 + 33.36 / 3600)),
    ('180 E', 180.0),
    ('180 W', -180.0),
    ('-180', -180.0),
    ('W 0 30', -0.5),
])
def test_parse_longitude_accepts(text, expected):
    assert parse_longitude(text) == pytest.approx(expected, abs=1e-12)


def test_the_three_formats_of_one_point_agree():
    dd = parse_latitude('38.9869')
    ddm = parse_latitude('38 59.214')
    dms = parse_latitude('38 59 12.84')
    assert ddm == pytest.approx(dd, abs=1e-9)
    assert dms == pytest.approx(dd, abs=1e-9)
    assert parse_lat_lon('38° 59\' 12.84" N', "76° 56.556' W") == pytest.approx(
        (38.9869, -76.9426), abs=1e-9)


@pytest.mark.parametrize('text', [
    '', '   ', 'abc', 'north', '38.9.9', '38,5', '.5', '5.', '1e1', '38 N 5',
    '38 59 12 5',              # four fields
    '38.5 30',                 # fractional degrees with minutes
    '38 59.5 12',              # fractional minutes with seconds
    '38 60', '38 59 60',       # minutes / seconds must be < 60
    '38 -30', '38 59 -1',      # no negative minutes / seconds
    '-38 S', '+38 N',          # sign AND hemisphere
    '38 E', '38 W',            # longitude hemisphere on a latitude
    '38 N S', 'N 38 S',        # two hemisphere letters
    'nan', 'inf', '-inf', '--5', '38 - 5',
    None, True, [38.9], {'lat': 1},
])
def test_parse_latitude_rejects_malformed_input(text):
    with pytest.raises(CoordinateFormatError):
        parse_latitude(text)


@pytest.mark.parametrize('text', ['76 N', '76 S', 'W', '76 W E', '7 6 5 4', '76 60'])
def test_parse_longitude_rejects_malformed_input(text):
    with pytest.raises(CoordinateFormatError):
        parse_longitude(text)


@pytest.mark.parametrize('text', ['90.0001', '91', '90 30', '90° 0\' 1" N', '-91', '100 S'])
def test_parse_latitude_rejects_out_of_range(text):
    with pytest.raises(InvalidCoordinateError, match='out of range'):
        parse_latitude(text)


@pytest.mark.parametrize('text', ['180.0001', '181', '180 30', '-181', '200 W'])
def test_parse_longitude_rejects_out_of_range(text):
    with pytest.raises(InvalidCoordinateError, match='out of range'):
        parse_longitude(text)


def test_parse_errors_are_invalid_coordinate_errors_and_mention_the_input():
    with pytest.raises(InvalidCoordinateError, match='38 60'):
        parse_latitude('38 60')
    assert issubclass(CoordinateFormatError, InvalidCoordinateError)
    assert issubclass(FrameRangeError, InvalidCoordinateError)
