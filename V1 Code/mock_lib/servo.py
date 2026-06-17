class Servo:
    def __init__(self, pin, *args, **kwargs):
        self._value = 0
    def enable(self): print("Servo enabled")
    def disable(self): print("Servo disabled")
    def value(self, v=None):
        if v is not None:
            self._value = v
            #print(f"Servo set to {v}")
        return self._value
    def to_min(self): print("Servo to min")
    def to_max(self): print("Servo to max")
    def to_mid(self): print("Servo to mid")
    def to_percent(self, *args): print("Servo to percent", args)
    def calibration(self, *args): return "MockCalibration"
    def mid_value(self): return 0

class ServoCluster:
    def __init__(self, *args, **kwargs): pass
    def enable_all(self): print("All servos enabled")
    def disable_all(self): print("All servos disabled")
    def all_to_min(self): print("All servos to min")
    def all_to_max(self): print("All servos to max")
    def all_to_mid(self): print("All servos to mid")
    def all_to_value(self, v): print(f"All servos to value {v}")
    def all_to_percent(self, *args): print("All servos to percent", args)
    def value(self, i, v, load=False): print(f"Servo {i} to {v}")
    def count(self): return 2
    def load(self): print("Servos loaded")

class servo2040:
    SERVO_1 = 0
    SERVO_2 = 1
    SERVO_3 = 2
    SERVO_4 = 3
    SERVO_7 = 6
    SERVO_8 = 7
    SERVO_9 = 8
    SERVO_10 = 9
    SERVO_13 = 12
    SERVO_14 = 13
    SERVO_15 = 14
    SERVO_16 = 15
    NUM_LEDS = 8
    LED_DATA = 0
    SHARED_ADC = 0
    CURRENT_GAIN = 1
    SHUNT_RESISTOR = 1
    CURRENT_OFFSET = 0
    ADC_ADDR_0 = 0
    ADC_ADDR_1 = 1
    ADC_ADDR_2 = 2
    USER_SW = 0
    SENSOR_1_ADDR = 0
    SENSOR_6_ADDR = 5
    VOLTAGE_GAIN = 1
    VOLTAGE_SENSE_ADDR = 0
    CURRENT_SENSE_ADDR = 0

ANGULAR = 0
LINEAR = 1
CONTINUOUS = 2

class Calibration:
    def __init__(self): pass
    def first_value(self, v): pass
    def last_value(self, v): pass
    def apply_two_pairs(self, a, b, c, d): pass
    def limit_to_calibration(self, a, b): pass