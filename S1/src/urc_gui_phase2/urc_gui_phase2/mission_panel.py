"""Mission editor panel: waypoint list + coordinate entry + edit buttons.

Talks only to a MissionController (signals in, methods out). Errors from the
model/frame are shown in the status line, never swallowed and never raised
into Qt's event loop.
"""

from typing import Optional

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QFileDialog, QHBoxLayout, QLabel, QLineEdit, QListWidget, QListWidgetItem, QPushButton,
    QVBoxLayout, QWidget)

from urc_gui_phase2.coordinate_convert import format_latitude, format_longitude
from urc_gui_phase2.coordinate_entry import CoordinateEntryWidget
from urc_gui_phase2.mission_controller import MissionController
from urc_gui_phase2.mission_model import InvalidCoordinateError, Status

_STATUS_MARK = {Status.PENDING: ' ', Status.ACTIVE: '>', Status.DONE: 'x'}
USER_ERRORS = (InvalidCoordinateError, LookupError, IndexError, ValueError, OSError)


ADD_MODE_OFF_TEXT = 'Add waypoint on click'
ADD_MODE_ON_TEXT = 'Click the map to place a waypoint  (click here to cancel)'
_ADD_MODE_OFF_STYLE = ''
_ADD_MODE_ON_STYLE = ('QPushButton { background: #ff9800; color: #000000; font-weight: bold; '
                      'border: 2px solid #e65100; padding: 4px; }')


class MissionPanel(QWidget):
    # True while the operator has armed "add waypoint on click". Off by default and one-shot:
    # the host turns it off after a successful add (see OperatorWindow._on_map_click_add).
    addModeChanged = pyqtSignal(bool)

    def __init__(self, controller: MissionController, parent=None):
        super().__init__(parent)
        self._ctl = controller
        self._list = QListWidget()
        self._name = QLineEdit()
        self._name.setPlaceholderText('Waypoint name')
        self._entry = CoordinateEntryWidget()
        self._status = QLabel()
        self._status.setWordWrap(True)

        self._add_mode = QPushButton(ADD_MODE_OFF_TEXT)
        self._add_mode.setCheckable(True)
        self._add = QPushButton('Add')
        self._update = QPushButton('Update selected')
        self._remove = QPushButton('Remove')
        self._up = QPushButton('Up')
        self._down = QPushButton('Down')
        self._activate = QPushButton('Set active')
        self._complete = QPushButton('Complete active')
        self._save = QPushButton('Save...')
        self._load = QPushButton('Load...')

        def row(*widgets):
            box = QHBoxLayout()
            for w in widgets:
                box.addWidget(w)
            return box

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel('Waypoints  (> active, x done)'))
        layout.addWidget(self._list, 1)
        layout.addWidget(self._add_mode)
        layout.addLayout(row(self._up, self._down, self._remove))
        layout.addLayout(row(self._activate, self._complete))
        layout.addWidget(self._name)
        layout.addWidget(self._entry)
        layout.addLayout(row(self._add, self._update))
        layout.addLayout(row(self._save, self._load))
        layout.addWidget(self._status)

        self._add_mode.toggled.connect(self._on_add_mode_toggled)
        self._add.clicked.connect(self._on_add)
        self._update.clicked.connect(self._on_update)
        self._remove.clicked.connect(lambda: self._run_on_selected(self._ctl.remove_waypoint))
        self._up.clicked.connect(lambda: self._run_on_selected(self._ctl.move_up))
        self._down.clicked.connect(lambda: self._run_on_selected(self._ctl.move_down))
        self._activate.clicked.connect(lambda: self._run_on_selected(self._ctl.activate))
        self._complete.clicked.connect(lambda: self._run(self._ctl.complete_active))
        self._save.clicked.connect(self._on_save)
        self._load.clicked.connect(self._on_load)
        self._entry.validityChanged.connect(self._sync_buttons)
        self._list.currentItemChanged.connect(self._on_list_selection)

        self._ctl.missionChanged.connect(self._refresh)
        self._ctl.selectionChanged.connect(self._on_controller_selection)
        self._refresh()

    # -- helpers ---------------------------------------------------------------

    def _run_on_selected(self, func):
        wp_id = self._ctl.selected_id
        if wp_id is None:
            self._say('Select a waypoint first.', ok=False)
            return None
        return self._run(func, wp_id)

    def _run(self, func, *args, **kwargs):
        """Call a controller command; report expected user errors in the status line."""
        try:
            result = func(*args, **kwargs)
        except USER_ERRORS as exc:
            self._say(str(exc), ok=False)
            return None
        self._say('', ok=True)
        return result

    @property
    def add_mode(self) -> bool:
        """True while a click on empty map should place a waypoint."""
        return self._add_mode.isChecked()

    def set_add_mode(self, enabled: bool) -> None:
        self._add_mode.setChecked(bool(enabled))

    def _on_add_mode_toggled(self, enabled: bool) -> None:
        self._add_mode.setText(ADD_MODE_ON_TEXT if enabled else ADD_MODE_OFF_TEXT)
        self._add_mode.setStyleSheet(_ADD_MODE_ON_STYLE if enabled else _ADD_MODE_OFF_STYLE)
        self.addModeChanged.emit(enabled)

    def report(self, text: str, ok: Optional[bool]) -> None:
        """Show a message in the status line (for other widgets that act on the mission).

        ok=True is green, ok=False is red, ok=None is a neutral hint.
        """
        self._say(text, ok)

    def _say(self, text: str, ok: Optional[bool]) -> None:
        colour = {True: '#1b5e20', False: '#b71c1c', None: '#616161'}[ok]
        self._status.setText(text)
        self._status.setStyleSheet(f'color: {colour};')

    def _sync_buttons(self, *_):
        has_sel = self._ctl.selected_id is not None
        valid = self._entry.is_valid
        self._add.setEnabled(valid)
        self._update.setEnabled(valid and has_sel)
        for button in (self._remove, self._up, self._down, self._activate):
            button.setEnabled(has_sel)
        self._complete.setEnabled(self._ctl.active_target() is not None)

    # -- actions ---------------------------------------------------------------

    def _on_add(self):
        coord = self._entry.coordinate()
        if coord is None:
            return
        name = self._name.text().strip() or self._ctl.default_waypoint_name()
        wp = self._run(self._ctl.add_waypoint, name, coord[0], coord[1])
        if wp is not None:
            self._ctl.select(wp.id)
            self._name.clear()
            self._entry.clear()

    def _on_update(self):
        coord, wp_id = self._entry.coordinate(), self._ctl.selected_id
        if coord is None or wp_id is None:
            return
        name = self._name.text().strip() or None
        self._run(self._ctl.edit_waypoint, wp_id, name=name, lat_deg=coord[0], lon_deg=coord[1])

    def _on_save(self):
        path, _ = QFileDialog.getSaveFileName(self, 'Save mission', 'mission.json',
                                              'Mission (*.json)')
        if path:
            self._run(self._ctl.save, path)

    def _on_load(self):
        path, _ = QFileDialog.getOpenFileName(self, 'Load mission', '', 'Mission (*.json)')
        if path:
            self._run(self._ctl.load, path)

    # -- controller -> view --------------------------------------------------

    def _refresh(self):
        selected = self._ctl.selected_id
        self._list.blockSignals(True)
        self._list.clear()
        for wp in self._ctl.model:
            item = QListWidgetItem(
                f'[{_STATUS_MARK[wp.status]}] {wp.name}   '
                f'{format_latitude(wp.lat_deg)}  {format_longitude(wp.lon_deg)}')
            item.setData(Qt.UserRole, wp.id)
            self._list.addItem(item)
            if wp.id == selected:
                self._list.setCurrentItem(item)
        self._list.blockSignals(False)
        self._sync_buttons()

    def _on_list_selection(self, current, _previous):
        wp_id = current.data(Qt.UserRole) if current is not None else None
        try:
            self._ctl.select(wp_id)
        except LookupError:
            return

    def _on_controller_selection(self, wp_id):
        self._refresh()
        if wp_id is not None:
            wp = self._ctl.model.get(wp_id)
            self._name.setText(wp.name)
            self._entry.set_coordinate(wp.lat_deg, wp.lon_deg)
        self._sync_buttons()
