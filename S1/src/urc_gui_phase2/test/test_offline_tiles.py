import json
import os

import pytest

from urc_gui_phase2.offline_tiles import (
    ENV_TILE_DIR, OfflineTileSource, TileSourceError, default_tile_dir, is_local_url)


def make_tiles(root, zooms=(12, 13), meta=None):
    for z in zooms:
        d = root / str(z) / '5'
        d.mkdir(parents=True)
        (d / '7.png').write_bytes(b'\x89PNG\r\n\x1a\n')
    if meta is not None:
        (root / 'metadata.json').write_text(json.dumps(meta))
    return root


@pytest.mark.parametrize('url', [
    'file:///home/x/tiles/12/1/2.png', 'file://localhost/x.png', 'qrc:///qtwebchannel/qwebchannel.js',
    'data:image/png;base64,AAAA', 'blob:null/1234', 'about:blank'])
def test_local_urls_allowed(url):
    assert is_local_url(url)


@pytest.mark.parametrize('url', [
    'http://tile.openstreetmap.org/1/2/3.png', 'https://unpkg.com/leaflet.js', 'ftp://x/y',
    'ws://localhost:1234/', 'http://127.0.0.1:8000/1/2/3.png', 'file://server/share/x.png',
    'gopher://x', 'HTTPS://EXAMPLE.COM/'])
def test_network_urls_blocked(url):
    assert not is_local_url(url)


def test_missing_directory_raises(tmp_path):
    with pytest.raises(TileSourceError, match='does not exist'):
        OfflineTileSource(str(tmp_path / 'nope'))


def test_empty_directory_raises(tmp_path):
    with pytest.raises(TileSourceError, match='no <z>/<x>/<y>.png'):
        OfflineTileSource(str(tmp_path))


def test_zoom_range_inferred_without_metadata(tmp_path):
    src = OfflineTileSource(str(make_tiles(tmp_path)))
    assert (src.min_zoom, src.max_zoom) == (12, 13)
    assert src.bounds is None and src.center is None
    assert src.covers(0, 0)  # unknown bounds: nothing is claimed to be outside


def test_metadata_is_used(tmp_path):
    meta = {'name': 'T', 'min_zoom': 12, 'max_zoom': 14, 'bounds': [38.0, -111.0, 39.0, -110.0],
            'center': [38.5, -110.5], 'tile_count': 3, 'attribution': 'osm'}
    src = OfflineTileSource(str(make_tiles(tmp_path, meta=meta)))
    assert src.max_zoom == 14 and src.center == (38.5, -110.5) and src.attribution == 'osm'
    assert src.covers(38.5, -110.5) and not src.covers(40, -110.5)
    assert 'zoom 12-14' in src.describe()


def test_bad_metadata_raises(tmp_path):
    make_tiles(tmp_path)
    (tmp_path / 'metadata.json').write_text('{not json')
    with pytest.raises(TileSourceError, match='cannot read'):
        OfflineTileSource(str(tmp_path))


def test_tile_path(tmp_path):
    src = OfflineTileSource(str(make_tiles(tmp_path)))
    assert src.tile_path(12, 5, 7) == str(tmp_path / '12' / '5' / '7.png')
    assert src.tile_path(12, 5, 8) is None


def test_url_template_is_file_scheme_and_encodes_path(tmp_path):
    root = make_tiles(tmp_path / 'my "tiles" dir')
    url = OfflineTileSource(str(root)).url_template
    assert url.startswith('file:///') and url.endswith('/{z}/{x}/{y}.png')
    assert ' ' not in url and '"' not in url
    assert is_local_url(url.replace('{z}', '12').replace('{x}', '5').replace('{y}', '7'))


def test_default_tile_dir_env_override(monkeypatch):
    monkeypatch.setenv(ENV_TILE_DIR, '/somewhere')
    assert default_tile_dir() == '/somewhere'
    monkeypatch.delenv(ENV_TILE_DIR)
    assert default_tile_dir().endswith(os.path.join('tiles', 'mdrs'))
