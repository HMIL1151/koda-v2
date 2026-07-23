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


koda = Robot()
# koda.wake()
# time.sleep(2)


# while True:
#     koda.zero_robot()

count = 0
while count < 10:
    walk_dir = Direction.FORWARDS
    walk_steps = random.randint(10, 10)
    
    turn_dir = Direction.BACKWARDS
    turn_steps = random.randint(10, 10)

    print(f"Walking {walk_dir} for {walk_steps} steps")
    koda.go_for_steps(walk_steps, walk_dir)
    time.sleep(0.5)
    
    print(f"Turning {turn_dir} for {turn_steps} steps")
    koda.go_for_steps(turn_steps, turn_dir)
    time.sleep(0.5)

    count = count + 1
    print()

koda.sleep()






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



