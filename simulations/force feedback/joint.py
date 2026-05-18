from units import *

class Joint():
    def __init__ (self, joint_type=JointType.PASSIVE, position: Coordinate | None = None, id=None):   
        self.position = position
        self.type = joint_type
        self.id = id
        self.links = []

        if self.type == JointType.DRIVEN_STATIC:
            if position is None:
                raise ValueError("Static Driven Joints must have a position")
    
    def set_position(self, position: Coordinate):
        self.position = position
    
    def get_force_vectors(self):
        if len(self.links) == 0:
            raise ValueError(f"no links attributed to joint: [{self.id}]")
        force_vectors = []
        for link in self.links:
            force_vectors.append(link.load)
        
        return force_vectors