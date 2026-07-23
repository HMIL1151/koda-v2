import inverse_kinematics
from constants import ZERO_X, ZERO_Y, ZERO_Z, SERVO_FREQUENCY
import bezier_curve
import math
from units import Direction


class Gait:
    CRAWL = 0
    TROT = 1
    GALLOP = 2

    
    
    STEP_HEIGHT = 42
    STEP_CURVE_DELTA = 0.2
    MANOUVRE_STEP_DISTANCE = 15
    MANOUVRE_SPEED = 50
    WALK_STEP_DISTANCE = 45
    WALK_SPEED = 200

    def __init__(self, gait_type):
        self.gait_type = gait_type
        self.direction = None
        self.speed = self.WALK_SPEED
        self.stance_steps = None
        self.swing_steps = None

    def calculate_gait(self, direction):
        step_distance = Gait.WALK_STEP_DISTANCE
        self.direction = direction
        gait_direction = direction
        

        if direction != Direction.FORWARDS and direction != Direction.BACKWARDS:
            step_distance = Gait.MANOUVRE_STEP_DISTANCE
            self.speed = self.MANOUVRE_SPEED

        if self.gait_type == Gait.CRAWL:
            self.speed = self.speed / 3
        self.stance_steps = int(SERVO_FREQUENCY * (step_distance / self.speed))
        

        if self.gait_type == Gait.CRAWL:
            self.swing_steps = int(self.stance_steps / 3)
        elif self.gait_type == Gait.TROT or self.gait_type == Gait.GALLOP :
            self.swing_steps = int(self.stance_steps)

        if direction == Direction.COUNTERCLOCKWISE:
            gait_direction = Direction.LEFT
        elif direction == Direction.CLOCKWISE:
            gait_direction = Direction.RIGHT
        

        

        path_points = bezier_curve.calculate_curve(step_distance, Gait.STEP_HEIGHT, Gait.STEP_CURVE_DELTA, self.stance_steps, self.swing_steps, gait_direction)
        #print("Path points:", path_points)

        servo_positions = inverse_kinematics.ik_points(path_points)
        self.speed = self.WALK_SPEED
        return servo_positions
    #except Exception as e:
           # raise ValueError("Error occurred during inverse kinematics: {}".format(e))

    def get_start_indices(self):
        
        if self.gait_type == Gait.CRAWL:
            return [0, 
                    int(self.swing_steps + self.stance_steps/3), 
                    int(self.swing_steps + 2*self.stance_steps/3), 
                    int(self.swing_steps)]
        elif self.gait_type == Gait.TROT:
            return [0, 
                    int(self.swing_steps), 
                    0, 
                    int(self.swing_steps)]
        elif self.gait_type == Gait.GALLOP:
            return [0, 
                    int(self.swing_steps // 4), 
                    int(self.swing_steps + self.swing_steps // 4), 
                    int(self.swing_steps)]
        else:
            raise ValueError("Invalid gait type")