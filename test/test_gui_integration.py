"""Coordinate entry, controller and panel, offscreen (no map needed)."""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest

pytest.importorskip('PyQt5.QtWidgets')

from PyQt5.QtCore import QCoreApplication, Qt  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

from urc_gui_phase2.coordinate_convert import (  # noqa: E402
    COORDINATE_FORMATS, LocalFrame, format_latitude, format_longitude, parse_latitude,
    parse_longitude)
from urc_gui_phase2.coordinate_entry import CoordinateEntryWidget  # noqa: E402
from urc_gui_phase2.mission_controller import MissionController  # noqa: E402
from urc_gui_phase2.mission_model import InvalidCoordinateError  # noqa: E402
from urc_gui_phase2.mission_panel import MissionPanel  # noqa: E402

# Must precede the first QApplication so QtWebEngine tests can share the process.
QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
_app = QApplication.instance() or QApplication([])
MDRS = (38.4058, -110.7919)


@pytest.mark.parametrize('fmt', COORDINATE_FORMATS)
@pytest.mark.parametrize('lat,lon', [MDRS, (-33.8688, 151.2093), (0.0, 0.0), (89.99999, 179.99999)])
def test_format_round_trips(fmt, lat, lon):
    assert parse_latitude(format_latitude(lat, fmt)) == pytest.approx(lat, abs=1e-4)
    assert parse_longitude(format_longitude(lon, fmt)) == pytest.approx(lon, abs=1e-4)


def test_format_carry_never_prints_60():
    assert format_latitude(38.9999999999, 'DDM') == "39\N{DEGREE SIGN} 0.0000' N"


def test_entry_validates_live_and_emits():
    w = CoordinateEntryWidget()
    got = []
    w.coordinateChanged.connect(lambda a, b: got.append((a, b)))
    w._lat_edit.setText('38 24 20.88 N')
    assert not w.is_valid and got == []
    w._lon_edit.setText('110 47.514 W')
    assert w.coordinate() == pytest.approx(MDRS, abs=1e-6)
    assert got
    w._lat_edit.setText('95')
    assert not w.is_valid and w.error_text
    assert w.coordinate() is None


def test_entry_set_coordinate_uses_selected_format():
    w = CoordinateEntryWidget()
    w.set_display_format('DMS')
    w.set_coordinate(*MDRS)
    assert '"' in w._lat_edit.text()
    assert w.coordinate() == pytest.approx(MDRS, abs=1e-5)


def test_controller_signals_and_active_edit():
    frame = LocalFrame(*MDRS)
    c = MissionController(frame=frame)
    events = []
    c.missionChanged.connect(lambda: events.append('m'))
    c.activeTargetChanged.connect(lambda wp: events.append(wp))
    wp = c.add_waypoint('A', 38.406, -110.792)
    c.activate(wp.id)
    assert events[-1].id == wp.id
    n = len(events)
    c.edit_waypoint(wp.id, lat_deg=38.407)   # editing the active target re-announces it
    assert events[-1].lat_deg == 38.407 and len(events) > n
    assert c.active_target_enu().north_m > 0


def test_controller_rejects_out_of_range_waypoint_and_changes_nothing():
    c = MissionController(frame=LocalFrame(*MDRS))
    with pytest.raises(InvalidCoordinateError):
        c.add_waypoint('far', 38.4, 110.79)   # wrong sign of longitude
    assert len(c.model) == 0


def test_panel_add_select_remove():
    c = MissionController()
    p = MissionPanel(c)
    p._name.setText('Start')
    p._entry._lat_edit.setText('38.4058 N')
    p._entry._lon_edit.setText('110.7919 W')
    p._add.click()
    assert len(c.model) == 1 and c.selected_id is not None
    assert p._list.count() == 1
    p._remove.click()
    assert len(c.model) == 0
    p._remove.click()   # nothing selected: message, no exception
    assert 'Select' in p._status.text() or not p._remove.isEnabled()
