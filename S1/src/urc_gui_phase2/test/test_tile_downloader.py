import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'tools'))

import pytest  # noqa: E402

import tile_downloader as td  # noqa: E402
from urc_gui_phase2.map_geometry import bbox_around  # noqa: E402


def test_default_plan_is_small_and_ordered():
    plan = td.plan_tiles(bbox_around(td.MDRS_LAT, td.MDRS_LON, 2500), 12, 17)
    assert 500 < len(plan) < 1000
    assert [t[0] for t in plan] == sorted(t[0] for t in plan)
    assert len(set(plan)) == len(plan)


def test_dry_run_downloads_nothing(tmp_path, capsys):
    out = tmp_path / 'tiles'
    assert td.main(['--dry-run', '--out', str(out)]) == 0
    assert not out.exists()
    assert 'total' in capsys.readouterr().out


def test_max_tiles_guard_refuses(tmp_path, capsys):
    assert td.main(['--max-zoom', '18', '--radius-km', '5', '--out', str(tmp_path / 't')]) == 2
    assert not (tmp_path / 't').exists()
    assert 'Refusing' in capsys.readouterr().err


def test_too_fast_against_osm_rejected():
    with pytest.raises(SystemExit):
        td.parse_args(['--delay', '0'])


def _fake_response(monkeypatch, body, headers=None, status=200):
    class Resp:
        def __init__(self):
            self.status = status
            self.headers = {'Content-Type': 'image/png', **(headers or {})}
        def __enter__(self): return self
        def __exit__(self, *a): return False
        def read(self): return body
    monkeypatch.setattr(td.urllib.request, 'urlopen', lambda *a, **k: Resp())


def test_fetch_rejects_non_png(monkeypatch):
    _fake_response(monkeypatch, b'<html>blocked</html>')
    with pytest.raises(td.DownloadError, match='not a PNG'):
        td.fetch('http://x/1.png', 'ua', 1)


def test_fetch_rejects_200_with_x_blocked_header(monkeypatch):
    # OSM serves its block page as a valid PNG with HTTP 200; only the header gives it away.
    _fake_response(monkeypatch, td.PNG_MAGIC + b'real-looking', {'X-Blocked': 'Access denied'})
    with pytest.raises(td.DownloadError, match='blocked'):
        td.fetch('http://x/1.png', 'ua', 1)


def test_fetch_rejects_wrong_content_type(monkeypatch):
    _fake_response(monkeypatch, td.PNG_MAGIC + b'x', {'Content-Type': 'text/html'})
    with pytest.raises(td.DownloadError, match='content-type'):
        td.fetch('http://x/1.png', 'ua', 1)


def test_fetch_accepts_real_png(monkeypatch):
    body = td.PNG_MAGIC + b'tile-bytes'
    _fake_response(monkeypatch, body)
    assert td.fetch('http://x/1.png', 'ua', 1) == body


def test_blocked_tiles_are_never_written(monkeypatch, tmp_path):
    _fake_response(monkeypatch, td.PNG_MAGIC + b'x', {'X-Blocked': 'yes'})
    monkeypatch.setattr(td.time, 'sleep', lambda s: None)
    out = tmp_path / 't'
    assert td.main(['--out', str(out), '--min-zoom', '12', '--max-zoom', '12']) == 1
    assert not list(out.rglob('*.png')) and not (out / 'metadata.json').exists()


def _write_tiles(root, body, n, zoom='13'):
    for i in range(n):
        d = root / zoom / str(i)
        d.mkdir(parents=True)
        (d / '1.png').write_bytes(body)


def test_verify_fails_on_known_placeholder(tmp_path, capsys):
    placeholder = tmp_path / '13' / '0'
    placeholder.mkdir(parents=True)
    # Fake the "known blocked" hash with a body we control.
    body = td.PNG_MAGIC + b'blocked-page'
    td.KNOWN_BLOCKED_MD5.add(td.hashlib.md5(body).hexdigest())
    try:
        (placeholder / '1.png').write_bytes(body)
        assert td.main(['--verify', '--out', str(tmp_path)]) == 1
        assert '1 bad' in capsys.readouterr().out
    finally:
        td.KNOWN_BLOCKED_MD5.discard(td.hashlib.md5(body).hexdigest())


def test_verify_identical_blank_tiles_only_warn(tmp_path, capsys):
    _write_tiles(tmp_path, td.PNG_MAGIC + b'blank-desert', 12)
    total, bad, warnings = td.verify_dir(str(tmp_path))
    assert total == 12 and bad == [] and len(warnings) == 1
    assert td.main(['--verify', '--out', str(tmp_path)]) == 0
    assert 'WARN' in capsys.readouterr().out


def test_verify_clean_dir_passes(tmp_path):
    for i in range(3):
        d = tmp_path / '13' / str(i)
        d.mkdir(parents=True)
        (d / '1.png').write_bytes(td.PNG_MAGIC + bytes([i]))
    assert td.main(['--verify', '--out', str(tmp_path)]) == 0


def test_write_atomic_leaves_no_part_file(tmp_path):
    path = tmp_path / '1' / '2' / '3.png'
    td.write_atomic(str(path), b'abc')
    assert path.read_bytes() == b'abc' and not (tmp_path / '1' / '2' / '3.png.part').exists()
