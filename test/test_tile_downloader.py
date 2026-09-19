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


def test_fetch_rejects_non_png(monkeypatch):
    class Resp:
        def __enter__(self): return self
        def __exit__(self, *a): return False
        def read(self): return b'<html>blocked</html>'
    monkeypatch.setattr(td.urllib.request, 'urlopen', lambda *a, **k: Resp())
    with pytest.raises(td.DownloadError, match='did not return a PNG'):
        td.fetch('http://x/1.png', 'ua', 1)


def test_write_atomic_leaves_no_part_file(tmp_path):
    path = tmp_path / '1' / '2' / '3.png'
    td.write_atomic(str(path), b'abc')
    assert path.read_bytes() == b'abc' and not (tmp_path / '1' / '2' / '3.png.part').exists()
