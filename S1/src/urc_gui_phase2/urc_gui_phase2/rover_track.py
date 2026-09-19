"""Pure-Python rover path and heading tracking.

The tracker deliberately has no ROS or Qt dependencies.  The ROS/controller
side supplies each fix in both WGS 84 and local REP 103 ENU coordinates, and
the map consumes the resulting geographic path and compass-style heading.
Keeping the calculation here makes it independently testable.
"""

import math
from typing import List, Optional, Tuple


DEFAULT_MIN_STEP_M = 0.5


class RoverTrack:
    """Keep meaningful rover fixes and derive heading from local ENU motion.

    Heading is expressed in degrees clockwise from north: north is 0 degrees,
    east is 90, south is 180, and west is 270.  Fixes closer than
    ``min_step_m`` to the last accepted fix do not change the path or heading;
    this prevents a stationary GNSS receiver from producing a flickering
    direction indicator.
    """

    def __init__(self, min_step_m: float = DEFAULT_MIN_STEP_M):
        min_step_m = float(min_step_m)
        if not math.isfinite(min_step_m) or min_step_m <= 0.0:
            raise ValueError('min_step_m must be a positive finite number')
        self._min_step_m = min_step_m
        self.reset()

    @property
    def points(self) -> Tuple[Tuple[float, float], ...]:
        """Return the accepted WGS 84 path as an immutable snapshot."""
        return tuple(self._points)

    @property
    def heading_deg(self) -> Optional[float]:
        """Return the latest compass heading, or ``None`` before movement."""
        return self._heading_deg

    def add_fix(self, lat_deg: float, lon_deg: float,
                east_m: float, north_m: float) -> bool:
        """Accept a position if it is the first fix or a meaningful movement.

        Returns ``True`` when the path changed.  Invalid non-finite values are
        rejected with ``ValueError`` so a bad sensor sample cannot corrupt the
        displayed history.
        """
        values = tuple(float(value) for value in (lat_deg, lon_deg, east_m, north_m))
        if not all(math.isfinite(value) for value in values):
            raise ValueError('rover fix values must be finite')
        lat_deg, lon_deg, east_m, north_m = values

        if self._last_enu is not None:
            delta_east = east_m - self._last_enu[0]
            delta_north = north_m - self._last_enu[1]
            if math.hypot(delta_east, delta_north) < self._min_step_m:
                return False

            # atan2(east, north) produces compass bearings rather than the
            # mathematical convention of measuring counter-clockwise from east.
            self._heading_deg = math.degrees(
                math.atan2(delta_east, delta_north)
            ) % 360.0

        self._points.append((lat_deg, lon_deg))
        self._last_enu = (east_m, north_m)
        return True

    def reset(self) -> None:
        """Clear the traveled path and remove the last known heading."""
        self._points: List[Tuple[float, float]] = []
        self._last_enu: Optional[Tuple[float, float]] = None
        self._heading_deg: Optional[float] = None
