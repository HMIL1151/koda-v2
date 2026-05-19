from __future__ import annotations
from units import *
from joint import *
from constants import *

class Link():
    def __init__ (self, joint_1: Joint, joint_2: Joint, length: float, angle: Angle | None = None, rate=spring_rate_N_per_mm, id=None):
        self.length = length
        self.upstream_joint = joint_1
        self.downstream_joint = joint_2
        self.joints = [self.upstream_joint, self.downstream_joint]
        self.spring_rate = rate
        self.length = length
        self.id = id
        self.driven = False
        self.position_set = False
        self.direction = self.get_direction()
        self.load = ForceVector.zero()

        for joint in self.joints:
            if joint.type == JointType.DRIVEN_STATIC:
                self.driven = True

                other_joint = Link.get_other_joint(self, joint)

                x = self.length * np.cos(angle.rad) + joint.position.x
                y = self.length * np.sin(angle.rad) + joint.position.y
                other_joint.set_position(Coordinate(x, y))
                self.position_set = True
                break

    def get_direction(self) -> Angle | None:

        for joint in self.joints:
            if joint.position is None:
                return None
            
       
        delta_x = self.downstream_joint.position.x - self.upstream_joint.position.x
        delta_y = self.downstream_joint.position.y - self.upstream_joint.position.y
        angle = Angle.from_radians(np.arctan2(delta_y, delta_x))

        return angle
    
    def update_direction(self):
        self.direction = self.get_direction()

    def update_length(self):
        if self.spring_rate is None:
            return
        
        self.length = self.length + self.load.mag.tension_N/self.spring_rate
    
    def set_load(self, load: Tension):
        self.update_length(load)
        self.update_direction()
        self.load = ForceVector(self.direction, load)

    @staticmethod
    def get_other_joint(link: Link, given_joint:Joint) -> Joint:
        return link.upstream_joint if given_joint == link.downstream_joint else link.downstream_joint