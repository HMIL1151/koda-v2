"""Tiny timing shim so the slave logic runs under both MicroPython and CPython.

MicroPython's monotonic tick helpers don't exist on desktop CPython; fall back to
time.monotonic there so the controller can be exercised against mock_lib.
"""

try:
    from time import ticks_ms, ticks_us, ticks_diff, sleep_ms  # MicroPython
except ImportError:                                            # CPython (emulation)
    import time as _time

    def ticks_ms():
        return int(_time.monotonic() * 1000)

    def ticks_us():
        return int(_time.monotonic() * 1_000_000)

    def ticks_diff(a, b):
        return a - b

    def sleep_ms(ms):
        _time.sleep(ms / 1000.0)
