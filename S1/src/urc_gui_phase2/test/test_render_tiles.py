import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'tools'))

import pytest  # noqa: E402

import render_tiles as rt  # noqa: E402
import tile_downloader as td  # noqa: E402

mapnik = pytest.importorskip('mapnik')

GEOJSON = '''{"type":"FeatureCollection","features":[
 {"type":"Feature","properties":{"highway":"primary","name":"Test Road"},
  "geometry":{"type":"LineString","coordinates":[[-110.80,38.40],[-110.78,38.41]]}},
 {"type":"Feature","properties":{"natural":"water"},
  "geometry":{"type":"Polygon","coordinates":[[[-110.80,38.40],[-110.79,38.40],[-110.79,38.41],[-110.80,38.40]]]}}
]}'''


def test_style_parses_at_every_zoom(tmp_path):
    path = tmp_path / 'd.geojson'
    path.write_text(GEOJSON)
    for z in range(12, 18):
        mp = mapnik.Map(256, 256)
        mapnik.load_map_from_string(mp, rt.style_xml(str(path), z))


def test_render_writes_valid_non_blocked_pngs(tmp_path):
    path = tmp_path / 'd.geojson'
    path.write_text(GEOJSON)
    out = tmp_path / 'tiles'
    assert rt.main(['--geojson', str(path), '--out', str(out), '--min-zoom', '13', '--max-zoom', '14']) == 0
    tiles = list(out.rglob('*.png'))
    assert tiles and (out / 'metadata.json').exists()
    total, bad, _ = td.verify_dir(str(out))
    assert total == len(tiles) and bad == []
    assert len({t.read_bytes() for t in tiles}) > 1  # the road tile differs from blank ones


def test_missing_geojson_is_an_error(tmp_path):
    assert rt.main(['--geojson', str(tmp_path / 'nope.geojson'), '--out', str(tmp_path / 't')]) == 1
