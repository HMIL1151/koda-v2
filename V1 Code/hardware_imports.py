import json
import os
from constants import EMULATION_MODE

if EMULATION_MODE:
    from mock_lib.servo import Servo, ServoCluster, servo2040, ANGULAR, LINEAR, CONTINUOUS, Calibration
    from mock_lib.mock_led import WS2812, NUM_LEDS, LED_DATA, RGBLED
else:
    try:
        from servo import Servo, ServoCluster, servo2040, ANGULAR, LINEAR, CONTINUOUS, Calibration
        # from pimoroni_ws2812 import WS2812
        # from servo2040 import NUM_LEDS, LED_DATA
        # from pimoroni import RGBLED

    except ImportError:
        raise ImportError("The real 'servo' module is only available on the Pico. Set 'use_mock_hardware' to True in settings.json to use the mock version on your PC.")