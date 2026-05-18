from __future__ import annotations
import numpy as np
from enum import Enum

class JointType(Enum):
    PASSIVE = 0
    DRIVEN_STATIC = 1

class Coordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Circle:
    def __init__(self, centre: Coordinate, radius: float):
        self.centre = centre
        self.radius = radius


class Tension:
    def __init__(self, tension: float):
        self.tension_N = tension

    @classmethod
    def from_newtons(cls, tension: float):
        return cls(tension)

class Angle:
    def __init__ (self, angle_rad: float):
        self.angle_rad = angle_rad

    @property
    def angle_deg(self) -> float:
        return np.degrees(self.angle_rad)
    
    @classmethod
    def from_degrees(cls, angle_deg: float) -> Angle:
        angle_rad = np.radians(angle_deg)
        return cls(angle_rad)

    @classmethod
    def from_radians(cls, angle_rad: float) -> Angle:
        return cls(angle_rad)

class ForceVector:
    def __init__(self, dir: Angle, mag: Tension):
        self.dir = dir
        self.mag = mag
    
    @classmethod
    def from_components(cls, x, y):
        dir = Angle.from_radians(np.arctan2(y, x))
        mag = Tension.from_newtons(np.hypot(x, y))
        return cls(dir, mag)
    
    @classmethod
    def zero(cls) -> ForceVector:
        return cls(Angle.from_degrees(0), Tension.from_newtons(0))
    
    @property
    def x_component(self):
        return self.mag.tension_N * np.cos(self.dir.angle_rad)
    
    @property
    def y_component(self):
        return self.mag.tension_N * np.sin(self.dir.angle_rad)
    
    @property
    def magnitude(self):
        return Tension.from_newtons(np.hypot(self.x_component, self.y_component))

    @staticmethod
    def find_equilibrium_vector(vectors: list[ForceVector]) -> ForceVector:    
        x_component_sum = 0
        y_component_sum = 0
        for vector in vectors:
            x_component_sum = x_component_sum + vector.x_component
            y_component_sum = y_component_sum + vector.y_component
    
        return ForceVector.from_components(-x_component_sum, -y_component_sum)
    
    @staticmethod
    def validate_equilibium(vectors: list[ForceVector]) -> bool:
        x_component_sum = 0
        y_component_sum = 0
        for vector in vectors:
            x_component_sum = x_component_sum + vector.x_component
            y_component_sum = y_component_sum + vector.y_component
        
        if (x_component_sum + y_component_sum) > 0.1:
            return False
        
        return True




        



    


        
        

        
