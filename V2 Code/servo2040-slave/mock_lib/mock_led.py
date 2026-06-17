"""Mock of the WS2812 LED strip for desktop testing."""


class WS2812:
    def __init__(self, num_leds, pio=0, sm=0, data=0):
        self.num_leds = num_leds
        self._pixels = [(0, 0, 0)] * num_leds

    def start(self):
        pass

    def set_rgb(self, index, r, g, b):
        if 0 <= index < self.num_leds:
            self._pixels[index] = (r, g, b)
