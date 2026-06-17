"""Status LEDs — a slimmed-down, fail-soft refactor of V1's led.py.

The slave's job is status, not animation: one colour per mode, plus whatever explicit LED
commands the master sends. If the LED hardware/driver isn't present the whole class
degrades to no-ops so the robot still runs headless.
"""

import config
import protocol

# Mode → (r, g, b)
_MODE_COLOUR = {
    protocol.RELAX:  (0, 0, 20),     # dim blue: asleep / torque off
    protocol.ACTIVE: (0, 40, 0),     # green: active
    protocol.HOLD:   (40, 30, 0),    # amber: holding
    protocol.SAFE:   (60, 0, 0),     # red: fail-safe
}


class StatusLeds:
    def __init__(self):
        self._strip = None
        try:
            if config.EMULATION_MODE:
                from mock_lib.mock_led import WS2812
                self._strip = WS2812(config.NUM_LEDS, 0, 0, 0)
            else:
                from plasma import WS2812
                from servo2040 import NUM_LEDS, LED_DATA
                self._strip = WS2812(NUM_LEDS, 0, 0, LED_DATA)
                self._strip.start()
        except Exception as exc:   # no LED hardware → run without it
            print("LEDs unavailable:", exc)
            self._strip = None

    def show_mode(self, mode):
        self.set_all(*_MODE_COLOUR.get(mode, (0, 0, 0)))

    def fault(self):
        self.set_all(80, 0, 0)

    def set_all(self, r, g, b):
        if not self._strip:
            return
        for i in range(config.NUM_LEDS):
            self._strip.set_rgb(i, r, g, b)

    def set(self, idx, r, g, b):
        if self._strip and 0 <= idx < config.NUM_LEDS:
            self._strip.set_rgb(idx, r, g, b)

    def clear(self):
        self.set_all(0, 0, 0)
