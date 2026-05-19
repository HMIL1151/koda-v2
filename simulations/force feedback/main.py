from imports import *

front_servo = Joint(JointType.DRIVEN_STATIC, Coordinate(-servo_seperation_mm/2, 0), id="front servo")
rear_servo = Joint(JointType.DRIVEN_STATIC, Coordinate(servo_seperation_mm/2, 0), id="rear servo")
front_knee = Joint(id="front knee")
rear_knee = Joint(id="rear knee")
foot = Joint(id="foot")

front_thigh = Link(front_servo, front_knee, thigh_length_mm, angle=Angle.from_degrees(180), rate=None, id="front thigh")
rear_thigh = Link(rear_servo, rear_knee, thigh_length_mm, angle=Angle.from_degrees(0), rate=None, id="rear thigh")
front_calf = Link(front_knee, foot, calf_length_mm, id="front calf")
rear_calf = Link(rear_knee, foot, calf_length_mm, id="rear calf")

leg = Linkage([front_thigh, rear_thigh, front_calf, rear_calf])

robot = Body(Mass.from_kg(3), wheelbase_mm, leg)
robot.draw_body()
robot.solve_robot()
print("new robot")
robot.draw_body()







