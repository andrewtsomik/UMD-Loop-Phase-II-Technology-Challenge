"""Editable, serializable mission model. Pure Python: no Qt, no ROS.

Design notes (each is a decision to defend):

* WGS84 lat/lon is the source of truth for every waypoint. ENU is *derived*
  from it through coordinate_convert.py. The ENU origin is the rover spawn
  point, which may not be known when the operator creates a waypoint (or may
  change between sessions), so storing ENU would bake in a frame that could
  later be wrong. Stored lat/lon stays valid whatever the anchor is, and a
  saved mission file is meaningful on its own.
* Status lives on the Waypoint, not in a separate "current index" or parallel
  list. A separate structure has to be kept in sync through every add, remove
  and reorder, and any missed case leaves the two disagreeing. With status on
  the waypoint, the status travels with the waypoint through all of those
  operations, and save/load gets it for free.
* Waypoints are immutable (frozen). Every edit builds a new, re-validated
  Waypoint, so an invalid waypoint cannot exist and nobody can bypass
  validation by assigning to a field. All mutation goes through MissionModel.
* Identity is a stable generated id, separate from the display name and from
  list position. Names may repeat or be renamed, and reordering never changes
  an id.
* Invariant: at most one waypoint is ACTIVE. Setting one active demotes any
  other to PENDING. Loading a file that violates this is rejected.
* Every failure raises a specific MissionError subclass. Nothing is silently
  ignored, and a failed operation leaves the model unchanged.

What this module deliberately does not decide: reordering does not touch
status, so a DONE waypoint can end up after a PENDING one. Whether that is
allowed is a policy for the GUI and the rover, not for the data model.
"""

import dataclasses
import json
import math
import os
import tempfile
import uuid
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Iterator, List, Optional, Tuple

SCHEMA_VERSION = 1


class MissionError(Exception):
    """Base class for all mission-model errors."""


class InvalidCoordinateError(MissionError, ValueError):
    """Latitude or longitude is missing, non-numeric, non-finite or out of range."""


class InvalidWaypointError(MissionError, ValueError):
    """A non-coordinate waypoint field (name, id, type, status) is invalid."""


class WaypointNotFoundError(MissionError, LookupError):
    """No waypoint has the requested id."""


class DuplicateWaypointError(MissionError, ValueError):
    """A waypoint with this id already exists."""


class WaypointIndexError(MissionError, IndexError):
    """A list index is outside the mission."""


class MissionFormatError(MissionError, ValueError):
    """A mission file or dict is malformed, or has an unsupported schema version."""


class TargetType(str, Enum):
    # The challenge requires only GNSS targets. This is an Enum so adding a
    # type later is one line, and unknown types in a file are rejected
    # instead of being accepted as free text.
    GNSS = 'GNSS'


class Status(str, Enum):
    PENDING = 'pending'
    ACTIVE = 'active'
    DONE = 'done'


def validate_coordinate(lat_deg: Any, lon_deg: Any) -> Tuple[float, float]:
    """Return (lat, lon) as floats, or raise InvalidCoordinateError.

    Range limits are inclusive: -90 <= lat <= 90, -180 <= lon <= 180.
    NaN and infinity fail the range comparison silently in plain Python
    (nan > 90 is False), so they are rejected explicitly. bool is rejected
    because isinstance(True, int) would otherwise let it through as 1.0.
    """
    values = []
    for label, value, limit in (('latitude', lat_deg, 90.0), ('longitude', lon_deg, 180.0)):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise InvalidCoordinateError(
                f'{label} must be a number, got {type(value).__name__}: {value!r}')
        value = float(value)
        if not math.isfinite(value):
            raise InvalidCoordinateError(f'{label} must be finite, got {value}')
        if not -limit <= value <= limit:
            raise InvalidCoordinateError(
                f'{label} {value} is out of range [-{limit:g}, {limit:g}]')
        values.append(value)
    return values[0], values[1]


@dataclass(frozen=True)
class Waypoint:
    id: str
    name: str
    lat_deg: float
    lon_deg: float
    target_type: TargetType = TargetType.GNSS
    status: Status = Status.PENDING

    def __post_init__(self):
        # Frozen dataclass: normalise through object.__setattr__.
        if not isinstance(self.id, str) or not self.id.strip():
            raise InvalidWaypointError(f'id must be a non-empty string, got {self.id!r}')
        if not isinstance(self.name, str) or not self.name.strip():
            raise InvalidWaypointError(f'name must be a non-empty string, got {self.name!r}')
        lat, lon = validate_coordinate(self.lat_deg, self.lon_deg)
        object.__setattr__(self, 'lat_deg', lat)
        object.__setattr__(self, 'lon_deg', lon)
        object.__setattr__(self, 'name', self.name.strip())
        try:
            object.__setattr__(self, 'target_type', TargetType(self.target_type))
            object.__setattr__(self, 'status', Status(self.status))
        except ValueError as exc:
            raise InvalidWaypointError(str(exc)) from None

    @property
    def coordinate(self) -> Tuple[float, float]:
        """(lat_deg, lon_deg) in WGS84."""
        return self.lat_deg, self.lon_deg

    def to_dict(self) -> Dict[str, Any]:
        return {
            'id': self.id,
            'name': self.name,
            'lat_deg': self.lat_deg,
            'lon_deg': self.lon_deg,
            'target_type': self.target_type.value,
            'status': self.status.value,
        }

    @classmethod
    def from_dict(cls, data: Any) -> 'Waypoint':
        if not isinstance(data, dict):
            raise MissionFormatError(f'waypoint must be an object, got {type(data).__name__}')
        missing = [k for k in ('id', 'name', 'lat_deg', 'lon_deg') if k not in data]
        if missing:
            raise MissionFormatError(f'waypoint is missing field(s): {", ".join(missing)}')
        try:
            return cls(
                id=data['id'],
                name=data['name'],
                lat_deg=data['lat_deg'],
                lon_deg=data['lon_deg'],
                target_type=data.get('target_type', TargetType.GNSS.value),
                status=data.get('status', Status.PENDING.value),
            )
        except MissionError as exc:
            raise MissionFormatError(f'invalid waypoint {data.get("id")!r}: {exc}') from exc


class MissionModel:
    """Ordered list of waypoints plus the operations the mission editor needs."""

    def __init__(self, name: str = ''):
        self.name = name
        self._waypoints: List[Waypoint] = []

    # ---- read access -------------------------------------------------------

    @property
    def waypoints(self) -> Tuple[Waypoint, ...]:
        """Read-only snapshot. Mutating the mission requires the methods below."""
        return tuple(self._waypoints)

    def __len__(self) -> int:
        return len(self._waypoints)

    def __iter__(self) -> Iterator[Waypoint]:
        return iter(tuple(self._waypoints))

    def __eq__(self, other) -> bool:
        if not isinstance(other, MissionModel):
            return NotImplemented
        return self.name == other.name and self._waypoints == other._waypoints

    def __repr__(self) -> str:
        return f'MissionModel(name={self.name!r}, waypoints={self._waypoints!r})'

    def get(self, waypoint_id: str) -> Waypoint:
        return self._waypoints[self.index_of(waypoint_id)]

    def index_of(self, waypoint_id: str) -> int:
        for i, wp in enumerate(self._waypoints):
            if wp.id == waypoint_id:
                return i
        raise WaypointNotFoundError(f'no waypoint with id {waypoint_id!r}')

    def get_active_target(self) -> Optional[Waypoint]:
        """The ACTIVE waypoint, or None if there is none (the invariant allows at most one)."""
        for wp in self._waypoints:
            if wp.status is Status.ACTIVE:
                return wp
        return None

    # ---- create / edit / delete -------------------------------------------

    def add(self, name: str, lat_deg: float, lon_deg: float,
            target_type: TargetType = TargetType.GNSS,
            waypoint_id: Optional[str] = None) -> Waypoint:
        """Append a new PENDING waypoint and return it.

        Raises InvalidCoordinateError / InvalidWaypointError on bad input and
        DuplicateWaypointError if an explicit id is already taken. On any
        error the mission is unchanged.
        """
        if waypoint_id is None:
            waypoint_id = self._new_id()
        elif any(wp.id == waypoint_id for wp in self._waypoints):
            raise DuplicateWaypointError(f'waypoint id {waypoint_id!r} already exists')
        waypoint = Waypoint(waypoint_id, name, lat_deg, lon_deg, target_type)
        self._waypoints.append(waypoint)
        return waypoint

    def edit(self, waypoint_id: str, *, name: Optional[str] = None,
             lat_deg: Optional[float] = None, lon_deg: Optional[float] = None) -> Waypoint:
        """Change the name and/or coordinate. Unspecified fields are kept.

        The result is validated as a whole before being stored, so a rejected
        edit leaves the original waypoint intact. Id, order and status are
        never changed by an edit.
        """
        index = self.index_of(waypoint_id)
        old = self._waypoints[index]
        new = dataclasses.replace(
            old,
            name=old.name if name is None else name,
            lat_deg=old.lat_deg if lat_deg is None else lat_deg,
            lon_deg=old.lon_deg if lon_deg is None else lon_deg,
        )
        self._waypoints[index] = new
        return new

    def remove(self, waypoint_id: str) -> Waypoint:
        """Remove by id and return the removed waypoint.

        Removing the ACTIVE waypoint leaves the mission with no active target;
        the model does not pick a replacement on its own.
        """
        return self._waypoints.pop(self.index_of(waypoint_id))

    def remove_at(self, index: int) -> Waypoint:
        self._check_index(index)
        return self._waypoints.pop(index)

    # ---- reorder -----------------------------------------------------------

    def move_to(self, waypoint_id: str, new_index: int) -> None:
        """Move a waypoint so it ends up at new_index (0-based)."""
        old_index = self.index_of(waypoint_id)
        self._check_index(new_index)
        self._waypoints.insert(new_index, self._waypoints.pop(old_index))

    def move_up(self, waypoint_id: str) -> None:
        """Move one place earlier. Raises WaypointIndexError if already first."""
        self.move_to(waypoint_id, self.index_of(waypoint_id) - 1)

    def move_down(self, waypoint_id: str) -> None:
        """Move one place later. Raises WaypointIndexError if already last."""
        self.move_to(waypoint_id, self.index_of(waypoint_id) + 1)

    # ---- status ------------------------------------------------------------

    def set_status(self, waypoint_id: str, status: Status) -> Waypoint:
        """Set a waypoint's status, keeping "at most one ACTIVE" true.

        Making a waypoint ACTIVE demotes the previously active one to PENDING.
        """
        index = self.index_of(waypoint_id)
        status = Status(status)
        updated = dataclasses.replace(self._waypoints[index], status=status)
        if status is Status.ACTIVE:
            for i, wp in enumerate(self._waypoints):
                if wp.status is Status.ACTIVE and i != index:
                    self._waypoints[i] = dataclasses.replace(wp, status=Status.PENDING)
        self._waypoints[index] = updated
        return updated

    def complete_active(self) -> Optional[Waypoint]:
        """Mark the active waypoint DONE and activate the next PENDING one.

        "Next" means the first PENDING waypoint after it in list order. Returns
        the newly active waypoint, or None if the mission is finished. Raises
        MissionError if nothing is active.
        """
        active = self.get_active_target()
        if active is None:
            raise MissionError('no active waypoint to complete')
        index = self.index_of(active.id)
        self.set_status(active.id, Status.DONE)
        for wp in self._waypoints[index + 1:]:
            if wp.status is Status.PENDING:
                return self.set_status(wp.id, Status.ACTIVE)
        return None

    # ---- save / load -------------------------------------------------------

    def to_dict(self) -> Dict[str, Any]:
        return {
            'schema_version': SCHEMA_VERSION,
            'name': self.name,
            'waypoints': [wp.to_dict() for wp in self._waypoints],
        }

    @classmethod
    def from_dict(cls, data: Any) -> 'MissionModel':
        """Build a model from a dict, or raise MissionFormatError.

        Validation happens on a fresh model, so a bad file can never leave a
        half-loaded mission behind.
        """
        if not isinstance(data, dict):
            raise MissionFormatError(f'mission must be an object, got {type(data).__name__}')
        version = data.get('schema_version')
        if version != SCHEMA_VERSION:
            raise MissionFormatError(
                f'unsupported schema_version {version!r} (this build reads {SCHEMA_VERSION})')
        raw_waypoints = data.get('waypoints')
        if not isinstance(raw_waypoints, list):
            raise MissionFormatError('"waypoints" must be a list')
        name = data.get('name', '')
        if not isinstance(name, str):
            raise MissionFormatError('"name" must be a string')

        model = cls(name)
        seen_ids = set()
        for raw in raw_waypoints:
            waypoint = Waypoint.from_dict(raw)
            if waypoint.id in seen_ids:
                raise MissionFormatError(f'duplicate waypoint id {waypoint.id!r}')
            seen_ids.add(waypoint.id)
            model._waypoints.append(waypoint)
        active = [wp.id for wp in model._waypoints if wp.status is Status.ACTIVE]
        if len(active) > 1:
            raise MissionFormatError(f'more than one active waypoint: {active}')
        return model

    def save(self, path: str) -> None:
        """Write the mission as JSON, atomically.

        Data goes to a temp file in the same directory, is flushed to disk,
        and then replaces the target in one step. A crash or a full disk
        mid-write therefore cannot leave the operator's previous file
        truncated.
        """
        directory = os.path.dirname(os.path.abspath(path))
        fd, tmp_path = tempfile.mkstemp(dir=directory, prefix='.mission-', suffix='.tmp')
        try:
            with os.fdopen(fd, 'w') as f:
                json.dump(self.to_dict(), f, indent=2, allow_nan=False)
                f.write('\n')
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_path, path)
        except BaseException:
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)
            raise

    @classmethod
    def load(cls, path: str) -> 'MissionModel':
        """Read a mission file. OSError (missing/unreadable file) is passed through."""
        with open(path, 'r') as f:
            try:
                data = json.load(f)
            except json.JSONDecodeError as exc:
                raise MissionFormatError(f'{path} is not valid JSON: {exc}') from exc
        return cls.from_dict(data)

    # ---- internals ---------------------------------------------------------

    def _check_index(self, index: int) -> None:
        if isinstance(index, bool) or not isinstance(index, int):
            raise WaypointIndexError(f'index must be an int, got {index!r}')
        if not 0 <= index < len(self._waypoints):
            raise WaypointIndexError(
                f'index {index} out of range for mission of {len(self._waypoints)} waypoints')

    def _new_id(self) -> str:
        existing = {wp.id for wp in self._waypoints}
        while True:
            candidate = uuid.uuid4().hex[:8]
            if candidate not in existing:
                return candidate
