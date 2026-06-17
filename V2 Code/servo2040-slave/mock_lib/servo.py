"""Mock of the Pimoroni `servo` module (Servo + servo2040 pin map) for desktop testing."""


class Servo:
    def __init__(self, pin, *args, **kwargs):
        self._pin = pin
        self._value = 0.0
        self._enabled = False

    def enable(self):
        self._enabled = True

    def disable(self):
        self._enabled = False

    def value(self, v=None):
        if v is not None:
            self._value = v
        return self._value


class servo2040:
    SERVO_1 = 0
    SERVO_2 = 1
    SERVO_3 = 2
    SERVO_4 = 3
    SERVO_5 = 4
    SERVO_6 = 5
    SERVO_7 = 6
    SERVO_8 = 7
    SERVO_9 = 8
    SERVO_10 = 9
    SERVO_11 = 10
    SERVO_12 = 11
    SERVO_13 = 12
    SERVO_14 = 13
    SERVO_15 = 14
    SERVO_16 = 15
    SERVO_17 = 16
    SERVO_18 = 17
    NUM_LEDS = 6
    LED_DATA = 18
