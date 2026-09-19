import math

import pytest

from urc_gui_phase2.map_geometry import (
    bbox_around, bounds_contain, latlon_to_tile, meters_per_pixel, pick_nearest,
    pixel_distance, tile_bounds, tiles_for_bbox)

MDRS = (38.4058, -110.7919)


def test_known_tile_from_osm_wiki():
    # OSM wiki worked example: lat 51.5, lon -0.12 at zoom 10 -> tile 511, 340
    assert latlon_to_tile(51.5, -0.12, 10) == (511, 340)


def test_zoom_zero_is_one_tile():
    assert latlon_to_tile(0, 0, 0) == (0, 0)
    assert latlon_to_tile(-85, 179.99, 0) == (0, 0)


def test_point_lies_inside_its_own_tile():
    for z in (5, 12, 17):
        x, y = latlon_to_tile(*MDRS, z)
        assert bounds_contain(tile_bounds(x, y, z), *MDRS)


def test_bbox_tiles_cover_bbox_and_zoom_grows_count():
    box = bbox_around(*MDRS, 2500)
    counts = []
    for z in (13, 15, 17):
        tiles = list(tiles_for_bbox(*box, z))
        counts.append(len(tiles))
        for corner in ((box[0], box[1]), (box[0], box[3]), (box[2], box[1]), (box[2], box[3])):
            assert latlon_to_tile(*corner, z) in tiles
    assert counts == sorted(counts) and counts[0] < counts[-1]


def test_bbox_is_roughly_square_in_metres():
    s, w, n, e = bbox_around(*MDRS, 1000)
    height = (n - s) * 111_320
    width = (e - w) * 111_320 * math.cos(math.radians(MDRS[0]))
    assert height == pytest.approx(2000, rel=1e-6)
    assert width == pytest.approx(2000, rel=1e-6)


def test_meters_per_pixel_matches_osm_table():
    # OSM zoom table: ~4.777 m/px at zoom 15 at the equator... scaled by cos(lat).
    assert meters_per_pixel(0, 15) == pytest.approx(4.777, rel=1e-3)
    assert meters_per_pixel(60, 15) == pytest.approx(4.777 / 2, rel=1e-3)


def test_pixel_distance_doubles_per_zoom_level():
    a, b = MDRS, (MDRS[0] + 0.001, MDRS[1] + 0.001)
    assert pixel_distance(*a, *b, 16) == pytest.approx(2 * pixel_distance(*a, *b, 15))
    assert pixel_distance(*a, *a, 15) == 0


def test_pick_nearest_within_threshold_only():
    lat, lon = MDRS
    # one pixel at zoom 17 in degrees of latitude, approximately
    deg_per_px = meters_per_pixel(lat, 17) / 111_320
    cands = [('near', lat + 5 * deg_per_px, lon), ('far', lat + 60 * deg_per_px, lon)]
    assert pick_nearest(lat, lon, cands, 17, 20) == 'near'
    assert pick_nearest(lat + 30 * deg_per_px, lon, [cands[0]], 17, 20) is None
    assert pick_nearest(lat, lon, [], 17, 20) is None


def test_pick_nearest_picks_closest_and_breaks_ties_to_first():
    lat, lon = MDRS
    d = 3 * meters_per_pixel(lat, 17) / 111_320
    assert pick_nearest(lat, lon, [('a', lat + 2 * d, lon), ('b', lat + d, lon)], 17, 50) == 'b'
    assert pick_nearest(lat, lon, [('a', lat + d, lon), ('b', lat + d, lon)], 17, 50) == 'a'


def test_threshold_is_in_pixels_not_metres():
    lat, lon = MDRS
    other = (lat + 0.0004, lon)  # ~44 m
    assert pick_nearest(lat, lon, [('w', *other)], 12, 20) == 'w'   # few px away when zoomed out
    assert pick_nearest(lat, lon, [('w', *other)], 17, 20) is None  # hundreds of px when zoomed in
