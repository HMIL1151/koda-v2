from servo2040 import NUM_LEDS, LED_DATA
from pimoroni import RGBLED
from pimoroni_ws2812 import WS2812
import time

class LEDArray:
    def __init__(self, num_leds=NUM_LEDS, led_data=LED_DATA):
        self.num_leds = num_leds
        self.led_strip = WS2812(led_data, num_leds)
        self.clear()

    def clear(self):
        """Turn off all LEDs."""
        self.led_strip.set_all(0, 0, 0)
        self.led_strip.show()

    def set_all(self, r, g, b):
        """Set all LEDs to the same color."""
        self.led_strip.set_all(r, g, b)
        self.led_strip.show()

    def set_led(self, index, r, g, b):
        """Set a single LED to a specific color."""
        if 0 <= index < self.num_leds:
            self.led_strip.set_rgb(index, r, g, b)
            self.led_strip.show()

    def rainbow(self, wait=0.05, cycles=1):
        """Display a rainbow animation across the LEDs."""
        for j in range(256 * cycles):
            for i in range(self.num_leds):
                rc_index = (i * 256 // self.num_leds) + j
                color = self._wheel(rc_index & 255)
                self.led_strip.set_rgb(i, *color)
            self.led_strip.show()
            time.sleep(wait)

    def chase(self, r, g, b, wait=0.05, cycles=1):
        """Chase a single color along the LED strip."""
        for c in range(cycles * self.num_leds):
            self.clear()
            self.led_strip.set_rgb(c % self.num_leds, r, g, b)
            self.led_strip.show()
            time.sleep(wait)

    def _wheel(self, pos):
        """Generate rainbow colors across 0-255 positions."""
        if pos < 85:
            return (int(pos * 3), int(255 - pos * 3), 0)
        elif pos < 170:
            pos -= 85
            return (int(255 - pos * 3), 0, int(pos * 3))
        else:
            pos -= 170
            return (0, int(pos * 3), int(255 - pos * 3))

# Example usage:
# led_array = LEDArray()
# led_array.set_all(255, 0, 0)  # All red
# led_array.rainbow()