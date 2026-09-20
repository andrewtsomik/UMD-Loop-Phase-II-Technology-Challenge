import math
import time


class TelemetryMonitor:
    """Classify one ROS data stream as waiting, active, or stale.

    The optional clock makes timeout behavior deterministic in unit tests. In
    production it defaults to Python's monotonic clock, which cannot jump when
    the computer's wall-clock time is corrected.
    """

    def __init__(self, stale_after_seconds=2.5, clock=time.monotonic):
        stale_after_seconds = float(stale_after_seconds)
        if not math.isfinite(stale_after_seconds) or stale_after_seconds <= 0.0:
            raise ValueError("stale_after_seconds must be positive and finite")
        self.stale_after_seconds = stale_after_seconds
        self._clock = clock
        self.last_received_time = None

    def mark_received(self):
        """Record that a telemetry message has arrived."""
        self.last_received_time = self._clock()

    def age_seconds(self):
        """Return the age of the latest message, or None if none arrived."""
        if self.last_received_time is None:
            return None

        return self._clock() - self.last_received_time

    def connection_state(self):
        """Return the connection state and latest message age."""
        age = self.age_seconds()

        if age is None:
            return "waiting", None

        if age <= self.stale_after_seconds:
            return "active", age

        return "stale", age
