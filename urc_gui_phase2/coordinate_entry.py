"""Coordinate entry widget: type a latitude/longitude in DD, DDM or DMS.

    CoordinateEntryWidget()
        coordinate()                 # (lat_deg, lon_deg), or None while invalid
        set_coordinate(lat, lon)     # fills both fields in the selected format
        clear()
        is_valid                     # property
        error_text                   # property: '' when valid
        coordinateChanged(float, float)   # signal: emitted when input becomes/stays valid
        validityChanged(bool)             # signal

Parsing is entirely coordinate_convert.parse_latitude/parse_longitude, so
the widget accepts exactly what those accept (any of DD/DDM/DMS, N/S/E/W or
sign, with or without degree marks), whatever the format selector says. The
selector only decides how set_coordinate() and the "Convert" button *display*
a value. Each field is checked as the operator types and shows its own error;
nothing is guessed or auto-corrected.

Qt only: no ROS, no map, no mission model.
"""

from typing import Optional, Tuple

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QComboBox, QFormLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QVBoxLayout, QWidget)

from urc_gui_phase2.coordinate_convert import (
    COORDINATE_FORMATS, format_latitude, format_longitude, parse_latitude,
    parse_longitude)
from urc_gui_phase2.mission_model import InvalidCoordinateError

_PLACEHOLDERS = {
    'DD': ('38.4058 N   or   38.4058', '110.7919 W   or   -110.7919'),
    'DDM': ('38 24.348 N', '110 47.514 W'),
    'DMS': ('38 24 20.88 N', '110 47 30.84 W'),
}
_ERROR_STYLE = 'QLineEdit { background: #ffebee; border: 1px solid #d32f2f; }'


class CoordinateEntryWidget(QWidget):
    coordinateChanged = pyqtSignal(float, float)
    validityChanged = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._lat: Optional[float] = None
        self._lon: Optional[float] = None
        self._valid = False
        self._lat_error = ''
        self._lon_error = ''

        self._format = QComboBox()
        self._format.addItems(COORDINATE_FORMATS)
        self._format.setToolTip('Display format for set_coordinate() and Convert. '
                                'Input in any format is always accepted.')
        self._lat_edit = QLineEdit()
        self._lon_edit = QLineEdit()
        self._lat_msg = QLabel()
        self._lon_msg = QLabel()
        for msg in (self._lat_msg, self._lon_msg):
            msg.setStyleSheet('color: #b71c1c;')
            msg.setWordWrap(True)
        self._convert = QPushButton('Convert')
        self._convert.setToolTip('Rewrite both fields in the selected format')

        form = QFormLayout()
        form.addRow('Latitude', self._lat_edit)
        form.addRow('', self._lat_msg)
        form.addRow('Longitude', self._lon_edit)
        form.addRow('', self._lon_msg)
        top = QHBoxLayout()
        top.addWidget(QLabel('Format'))
        top.addWidget(self._format)
        top.addWidget(self._convert)
        top.addStretch(1)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addLayout(top)
        layout.addLayout(form)

        self._lat_edit.textChanged.connect(self._revalidate)
        self._lon_edit.textChanged.connect(self._revalidate)
        self._format.currentTextChanged.connect(self._on_format_changed)
        self._convert.clicked.connect(self._reformat)
        self._on_format_changed(self._format.currentText())
        self._revalidate()

    # -- public API ----------------------------------------------------------

    @property
    def is_valid(self) -> bool:
        return self._valid

    @property
    def error_text(self) -> str:
        return '; '.join(e for e in (self._lat_error, self._lon_error) if e)

    @property
    def display_format(self) -> str:
        return self._format.currentText()

    def set_display_format(self, fmt: str) -> None:
        if fmt not in COORDINATE_FORMATS:
            raise ValueError(f'unknown coordinate format {fmt!r}')
        self._format.setCurrentText(fmt)

    def coordinate(self) -> Optional[Tuple[float, float]]:
        return (self._lat, self._lon) if self._valid else None

    def set_coordinate(self, lat_deg: float, lon_deg: float) -> None:
        """Show a coordinate in the selected format. Raises on an out-of-range value."""
        lat_text = format_latitude(lat_deg, self.display_format)
        lon_text = format_longitude(lon_deg, self.display_format)
        self._set_texts(lat_text, lon_text)

    def clear(self) -> None:
        self._set_texts('', '')

    def set_read_only(self, read_only: bool) -> None:
        for widget in (self._lat_edit, self._lon_edit):
            widget.setReadOnly(read_only)
        self._convert.setEnabled(not read_only)

    # -- internals -----------------------------------------------------------

    def _set_texts(self, lat_text: str, lon_text: str) -> None:
        # Block per-field signals so listeners see one consistent update.
        for edit, text in ((self._lat_edit, lat_text), (self._lon_edit, lon_text)):
            edit.blockSignals(True)
            edit.setText(text)
            edit.blockSignals(False)
        self._revalidate()

    def _on_format_changed(self, fmt: str) -> None:
        lat_hint, lon_hint = _PLACEHOLDERS[fmt]
        self._lat_edit.setPlaceholderText(lat_hint)
        self._lon_edit.setPlaceholderText(lon_hint)

    def _reformat(self) -> None:
        if self._valid:
            self.set_coordinate(self._lat, self._lon)

    def _revalidate(self) -> None:
        was_valid = self._valid
        old = (self._lat, self._lon)
        self._lat, self._lat_error = self._parse(self._lat_edit, self._lat_msg, parse_latitude)
        self._lon, self._lon_error = self._parse(self._lon_edit, self._lon_msg, parse_longitude)
        self._valid = self._lat is not None and self._lon is not None
        if self._valid:
            if not was_valid or old != (self._lat, self._lon):
                self.coordinateChanged.emit(self._lat, self._lon)
        if self._valid != was_valid:
            self.validityChanged.emit(self._valid)

    @staticmethod
    def _parse(edit: QLineEdit, msg: QLabel, parser):
        """Return (value or None, error text). An empty field is 'incomplete', not an error."""
        text = edit.text().strip()
        if not text:
            msg.setText('')
            edit.setStyleSheet('')
            return None, ''
        try:
            value = parser(text)
        except InvalidCoordinateError as exc:
            message = str(exc)
            msg.setText(message)
            edit.setStyleSheet(_ERROR_STYLE)
            return None, message
        msg.setText('')
        edit.setStyleSheet('')
        return value, ''
