NUM_LEDS = 12
LED_DATA = 0

class WS2812:
    def __init__(self, led_data, num_leds):
        self.led_data = led_data
        self.num_leds = num_leds
        self.leds = [(0, 0, 0)] * num_leds

    def set_all(self, r, g, b):
        self.leds = [(r, g, b)] * self.num_leds
        print(f"[MOCK] Set all LEDs to ({r}, {g}, {b})")

    def set_rgb(self, index, r, g, b):
        if 0 <= index < self.num_leds:
            self.leds[index] = (r, g, b)
            print(f"[MOCK] Set LED {index} to ({r}, {g}, {b})")

    def show(self):
        print(f"[MOCK] LED states: {self.leds}")

class RGBLED:
    def __init__(self, *args, **kwargs):
        pass