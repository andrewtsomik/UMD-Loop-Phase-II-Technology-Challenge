"""The adapter: the one place that joins the mission model, the local frame, the
map and the ROS node. Qt signals out, plain method calls in; no ROS imports.

    MissionController(model=None, frame=None)
        # commands (each raises the mission_model / coordinate_convert error
        # on bad input and leaves the mission unchanged)
        add_waypoint(name, lat, lon, target_type=GNSS) -> Waypoint
        default_waypoint_name() -> str      # 'WP<n+1>', the name used when none is typed
        edit_waypoint(id, name=None, lat=None, lon=None)
        remove_waypoint(id) / move_up(id) / move_down(id)
        activate(id) / complete_active()
        save(path) / load(path)
        set_frame(frame) / set_rover_fix(lat, lon)
        # queries
        model, frame, rover_latlon, rover_enu(), active_target(), active_target_enu()
        # signals
        missionChanged()            any add/edit/remove/reorder/status/load
        activeTargetChanged(object) Waypoint or None (only when the active waypoint changes)
        roverMoved(float, float)    lat, lon
        frameChanged(object)        LocalFrame or None
        selectionChanged(object)    waypoint id or None (shared by map and list)

Consumers (map widget, list panel, ROS node, anyone else's GUI) subscribe to
these signals and call these methods; nothing else needs to reach into the
model. The controller never touches widgets, so another GUI can drive or
observe it without importing Qt widgets.
"""

from typing import Optional, Tuple

from PyQt5.QtCore import QObject, pyqtSignal

from urc_gui_phase2.coordinate_convert import ENU, LocalFrame
from urc_gui_phase2.mission_model import MissionModel, Status, TargetType, Waypoint


class MissionController(QObject):
    missionChanged = pyqtSignal()
    activeTargetChanged = pyqtSignal(object)
    roverMoved = pyqtSignal(float, float)
    frameChanged = pyqtSignal(object)
    selectionChanged = pyqtSignal(object)

    def __init__(self, model: Optional[MissionModel] = None,
                 frame: Optional[LocalFrame] = None, parent=None):
        super().__init__(parent)
        self._model = model if model is not None else MissionModel()
        self._frame = frame
        self._rover: Optional[Tuple[float, float]] = None
        self._selected: Optional[str] = None
        self._active_wp: Optional[Waypoint] = self._model.get_active_target()

    # -- queries -------------------------------------------------------------

    @property
    def model(self) -> MissionModel:
        """Read it freely; mutate only through the controller so signals fire."""
        return self._model

    @property
    def frame(self) -> Optional[LocalFrame]:
        return self._frame

    @property
    def rover_latlon(self) -> Optional[Tuple[float, float]]:
        return self._rover

    @property
    def selected_id(self) -> Optional[str]:
        return self._selected

    def active_target(self) -> Optional[Waypoint]:
        return self._model.get_active_target()

    def rover_enu(self) -> Optional[ENU]:
        if self._frame is None or self._rover is None:
            return None
        return self._frame.to_enu(*self._rover)

    def active_target_enu(self) -> Optional[ENU]:
        """The active waypoint in the local frame, or None (no frame / no active target)."""
        wp = self.active_target()
        if self._frame is None or wp is None:
            return None
        return self._frame.to_enu(wp.lat_deg, wp.lon_deg)

    # -- mission commands ----------------------------------------------------

    def default_waypoint_name(self) -> str:
        """Name for a waypoint added without one (typed-entry panel and map click alike)."""
        return f'WP{len(self._model) + 1}'

    def add_waypoint(self, name: str, lat_deg: float, lon_deg: float,
                     target_type: TargetType = TargetType.GNSS) -> Waypoint:
        self._check_in_range(lat_deg, lon_deg)
        wp = self._model.add(name, lat_deg, lon_deg, target_type)
        self._changed()
        return wp

    def edit_waypoint(self, waypoint_id: str, *, name: Optional[str] = None,
                      lat_deg: Optional[float] = None, lon_deg: Optional[float] = None) -> Waypoint:
        old = self._model.get(waypoint_id)
        self._check_in_range(old.lat_deg if lat_deg is None else lat_deg,
                             old.lon_deg if lon_deg is None else lon_deg)
        wp = self._model.edit(waypoint_id, name=name, lat_deg=lat_deg, lon_deg=lon_deg)
        self._changed()
        return wp

    def remove_waypoint(self, waypoint_id: str) -> Waypoint:
        wp = self._model.remove(waypoint_id)
        if self._selected == waypoint_id:
            self.select(None)
        self._changed()
        return wp

    def move_up(self, waypoint_id: str) -> None:
        self._model.move_up(waypoint_id)
        self._changed()

    def move_down(self, waypoint_id: str) -> None:
        self._model.move_down(waypoint_id)
        self._changed()

    def activate(self, waypoint_id: str) -> None:
        self._model.set_status(waypoint_id, Status.ACTIVE)
        self._changed()

    def complete_active(self) -> Optional[Waypoint]:
        nxt = self._model.complete_active()
        self._changed()
        return nxt

    def save(self, path: str) -> None:
        self._model.save(path)

    def load(self, path: str) -> None:
        """Replace the mission from a file. A bad file raises and changes nothing."""
        self._model = MissionModel.load(path)
        if self._selected is not None and all(w.id != self._selected for w in self._model):
            self.select(None)
        self._changed()

    def select(self, waypoint_id: Optional[str]) -> None:
        if waypoint_id is not None:
            self._model.get(waypoint_id)   # WaypointNotFoundError if unknown
        if waypoint_id != self._selected:
            self._selected = waypoint_id
            self.selectionChanged.emit(waypoint_id)

    # -- frame and rover -----------------------------------------------------

    def set_frame(self, frame: Optional[LocalFrame]) -> None:
        self._frame = frame
        self.frameChanged.emit(frame)
        # The active target's ENU depends on the frame, so consumers re-read it.
        self.activeTargetChanged.emit(self._active_wp)

    def set_rover_fix(self, lat_deg: float, lon_deg: float) -> None:
        lat, lon = float(lat_deg), float(lon_deg)
        if (lat, lon) != self._rover:
            self._rover = (lat, lon)
            self.roverMoved.emit(lat, lon)

    # -- internals -----------------------------------------------------------

    def _check_in_range(self, lat_deg: float, lon_deg: float) -> None:
        """Reject a waypoint the frame could not convert (typo / wrong sign)."""
        if self._frame is not None:
            self._frame.to_enu(lat_deg, lon_deg)   # FrameRangeError if too far

    def _changed(self) -> None:
        self.missionChanged.emit()
        # Waypoint is a frozen dataclass, so == also catches an edited coordinate.
        active = self._model.get_active_target()
        if active != self._active_wp:
            self._active_wp = active
            self.activeTargetChanged.emit(active)
