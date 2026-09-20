import time


class TelemetryMonitor:
    """Tracks when telemetry was last received."""

    def __init__(self, stale_after_seconds=2.5):
        self.stale_after_seconds = stale_after_seconds
        self.last_received_time = None

    def mark_received(self):
        """Record that a telemetry message has arrived."""
        self.last_received_time = time.monotonic()

    def age_seconds(self):
        """Return the age of the latest message, or None if none arrived."""
        if self.last_received_time is None:
            return None

        return time.monotonic() - self.last_received_time

    def connection_state(self):
        """Return the connection state and latest message age."""
        age = self.age_seconds()

        if age is None:
            return "waiting", None

        if age <= self.stale_after_seconds:
            return "active", age

        return "stale", age