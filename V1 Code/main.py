import gc
from robot import Robot
from gait import Gait
import random
from units import Speed, Direction
import misc_functions
import time
import orientation
import inverse_kinematics
import constants
from machine import Pin
from pimoroni import Analog, AnalogMux
from servo import servo2040


# Set up the shared analog input and multiplexer
sen_adc = Analog(servo2040.SHARED_ADC)
mux = AnalogMux(servo2040.ADC_ADDR_0, servo2040.ADC_ADDR_1, servo2040.ADC_ADDR_2,
                muxed_pin=Pin(servo2040.SHARED_ADC))

# Configure sensor 1 with pull-down for the potentiometer
mux.configure_pull(servo2040.SENSOR_1_ADDR, Pin.PULL_DOWN)

walk_directions = [Direction.FORWARDS, Direction.BACKWARDS]
turn_directions = [Direction.CLOCKWISE, Direction.COUNTERCLOCKWISE]

koda = Robot()
# koda.wake()
# time.sleep(2)
koda.rotation_test(50)

time.sleep(1)

koda.sleep()

# count = 0
# while count < 10:
#     walk_dir = walk_directions[random.randint(0, 1)]
#     walk_steps = random.randint(10, 20)
    
#     turn_dir = turn_directions[random.randint(0, 1)]
#     turn_steps = random.randint(5, 10)

#     print(f"Walking {walk_dir} for {walk_steps} steps")
#     koda.go_for_steps(walk_steps, walk_dir)
#     time.sleep(0.5)
    
#     print(f"Turning {turn_dir} for {turn_steps} steps")
#     koda.go_for_steps(turn_steps, turn_dir)
#     time.sleep(0.5)

#     count = count + 1
#     print()

# koda.sleep()






# while True:
#     koda.manual_servo_control([0, 0, 0])

#koda.stand()



# direction = Direction.RIGHT
# while True:
#     mux.select(servo2040.SENSOR_1_ADDR)
#     voltage = sen_adc.read_voltage()
#     speed = int(misc_functions.map_value(voltage, 0, 3.3, 50, 300))
#     koda.set_speed(Speed.in_mm_per_second(speed))
#     print(f"Speed: {speed:.1f} mm/s")
    
#     if direction == Direction.RIGHT:
#         direction = Direction.LEFT
#     else:
#         direction = Direction.RIGHT

#     koda.set_gait(Gait.TROT, direction)
#     koda.go_for_steps(10)
    
# while True:
    # for direction in [Direction.LEFT, Direction.RIGHT]:
    #     steps = 10
    #     if direction != Direction.FORWARDS and direction != Direction.BACKWARDS:
    #         steps = 5
    
    #     koda.go_for_steps(steps, direction)


    # time.sleep(2)

    # koda.go_for_steps(10, Direction.FORWARDS)



