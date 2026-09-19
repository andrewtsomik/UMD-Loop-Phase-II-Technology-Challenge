"""Structured, exportable event log for an operator-console session.

The logger is deliberately independent of ROS and Qt. GUI callbacks record
small, meaningful events, and the operator exports them as a CSV file that can
be opened in a spreadsheet or parsed by a script.
"""

import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import tempfile
import time


CSV_FIELDS = (
    "timestamp_utc",
    "elapsed_s",
    "category",
    "event",
    "details_json",
)


class MissionEventLog:
    """Collect timestamped mission events and atomically export them to CSV."""

    def __init__(self, wall_clock=time.time, monotonic_clock=time.monotonic):
        self._wall_clock = wall_clock
        self._monotonic_clock = monotonic_clock
        self._started_at = monotonic_clock()
        self._events = []

    @property
    def events(self):
        """Return an immutable snapshot of the events recorded so far."""
        return tuple(dict(event) for event in self._events)

    def record(self, category, event, **details):
        """Append one event and return its immutable-by-convention dictionary."""
        category = str(category).strip()
        event = str(event).strip()
        if not category or not event:
            raise ValueError("log category and event must not be empty")

        wall_seconds = float(self._wall_clock())
        elapsed_s = float(self._monotonic_clock()) - self._started_at
        if not (math.isfinite(wall_seconds) and math.isfinite(elapsed_s)):
            raise ValueError("log clocks must return finite values")

        # Serializing now catches unsupported detail values at the call site,
        # instead of surprising the operator only when Export is pressed.
        details_json = json.dumps(
            details,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
        )
        timestamp = datetime.fromtimestamp(
            wall_seconds,
            tz=timezone.utc,
        ).isoformat(timespec="milliseconds").replace("+00:00", "Z")
        row = {
            "timestamp_utc": timestamp,
            "elapsed_s": round(elapsed_s, 3),
            "category": category,
            "event": event,
            "details_json": details_json,
        }
        self._events.append(row)
        return dict(row)

    def export_csv(self, path):
        """Atomically write all current events and return the destination.

        The temporary file is created next to the destination. ``os.replace``
        then makes the finished CSV appear in one operation, so a failed write
        cannot leave a partially written mission log at the chosen path.
        """
        destination = Path(path).expanduser()
        if not destination.name:
            raise ValueError("log destination must include a file name")
        destination.parent.mkdir(parents=True, exist_ok=True)

        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{destination.name}.",
            suffix=".tmp",
            dir=destination.parent,
            text=True,
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
                writer.writeheader()
                writer.writerows(self._events)
            os.replace(temporary_name, destination)
        except Exception:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise
        return destination
