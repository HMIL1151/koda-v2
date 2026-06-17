"""Link watchdog. The slave must never keep holding a stale pose (or worse, keep driving)
after the master goes quiet. Any valid frame feeds the watchdog; if none arrives within
LINK_TIMEOUT_MS it reports expired, and main switches to the SAFE fail-safe.
"""

from compat import ticks_ms, ticks_diff
import config


class Watchdog:
    def __init__(self, timeout_ms=config.LINK_TIMEOUT_MS):
        self._timeout = timeout_ms
        self._last_ok = ticks_ms()
        self._tripped = False

    def feed(self):
        """Call on every valid frame from the master."""
        self._last_ok = ticks_ms()
        self._tripped = False

    def expired(self):
        return ticks_diff(ticks_ms(), self._last_ok) > self._timeout

    def trip_once(self):
        """True exactly once per timeout event, so faults aren't spammed every loop."""
        if self.expired() and not self._tripped:
            self._tripped = True
            return True
        return False
