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
leg.set_link_loads((
    (front_calf, Tension.from_newtons(-10)), 
    (rear_calf, Tension.from_newtons(5))
    ))

#leg.draw_linkage()
force_vectors = foot.get_force_vectors()
reaction = ForceVector.find_equilibrium_vector(force_vectors)
print(f"Foot Reaction: Rx = {reaction.x_component}, Ry = {reaction.y_component}")

