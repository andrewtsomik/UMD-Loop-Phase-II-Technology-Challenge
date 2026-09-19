"""Unit tests for structured mission-event recording and CSV export."""

import csv
import json

import pytest

from urc_rover_console.mission_event_log import MissionEventLog


class SequenceClock:
    """Return controlled values in order."""

    def __init__(self, *values):
        self._values = iter(values)

    def __call__(self):
        return next(self._values)


def test_record_keeps_timestamp_category_event_and_structured_details():
    log = MissionEventLog(
        wall_clock=SequenceClock(0.0),
        monotonic_clock=SequenceClock(10.0, 11.25),
    )

    row = log.record("operator_command", "sent", command="START_MISSION")

    assert row["timestamp_utc"] == "1970-01-01T00:00:00.000Z"
    assert row["elapsed_s"] == 1.25
    assert row["category"] == "operator_command"
    assert row["event"] == "sent"
    assert json.loads(row["details_json"]) == {"command": "START_MISSION"}
    assert log.events == (row,)


def test_export_csv_round_trips_details_and_replaces_existing_file(tmp_path):
    monotonic = SequenceClock(5.0, 5.5, 6.0)
    wall = SequenceClock(100.0, 101.0)
    log = MissionEventLog(wall_clock=wall, monotonic_clock=monotonic)
    log.record("mission", "started", waypoint_count=2)
    log.record("target_result", "reached", name="Science Site")
    destination = tmp_path / "mission.csv"
    destination.write_text("incomplete old content", encoding="utf-8")

    assert log.export_csv(destination) == destination

    with destination.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert [row["event"] for row in rows] == ["started", "reached"]
    assert json.loads(rows[1]["details_json"]) == {"name": "Science Site"}
    assert not list(tmp_path.glob("*.tmp"))


@pytest.mark.parametrize(
    ("category", "event"),
    [("", "sent"), ("operator_command", " ")],
)
def test_record_rejects_empty_names(category, event):
    log = MissionEventLog()

    with pytest.raises(ValueError):
        log.record(category, event)


def test_record_rejects_non_json_details_before_export():
    log = MissionEventLog()

    with pytest.raises(TypeError):
        log.record("mission", "bad_detail", unsupported=object())
