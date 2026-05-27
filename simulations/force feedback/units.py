from __future__ import annotations
import numpy as np
from enum import Enum
from constants import *

class JointType(Enum):
    PASSIVE = 0
    DRIVEN_STATIC = 1

class Coordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    @classmethod
    def origin(cls):
        return cls(0, 0)
    
    @staticmethod
    def find_end_point(known_point: Coordinate, distance: float, direction: Angle) -> Coordinate:
        dx = distance * np.cos(direction.rad)
        dy = distance * np.sin(direction.rad)

        return Coordinate(known_point.x + dx, known_point.y + dy)

    @staticmethod
    def get_lower_point(points: list[Coordinate]) -> Coordinate:
        lowest = points[0]

        for point in points:
            if point.y < lowest.y:
                lowest = point
        
        return lowest

    @staticmethod
    def distance_between_points(point1: Coordinate, point2:Coordinate) -> Coordinate:
        return Coordinate(point1.x - point2.x, point1.y - point2.y)
    
    @staticmethod
    def translate(point: Coordinate, delta: Coordinate) -> Coordinate:
        return Coordinate(point.x + delta.x, point.y + delta.y)
    
    @staticmethod
    def rotate(point: Coordinate, centre: Coordinate, angle: Angle) -> Coordinate:
        dx = point.x - centre.x
        dy = point.y - centre.y

        rotated_x = dx * np.cos(-angle.rad) - dy * np.sin(-angle.rad)
        rotated_y = dx * np.sin(-angle.rad) + dy * np.cos(-angle.rad)

        return Coordinate(centre.x + rotated_x, centre.y + rotated_y)

    @staticmethod
    def negative(point: Coordinate) -> Coordinate:
        return Coordinate(-point.x, -point.y)
    
    @staticmethod
    def get_angle_from_points(start: Coordinate, end: Coordinate) -> Angle:
        dx = end.x - start.x
        dy = end.y - start.y
        return Angle.from_radians(np.atan2(dy, dx))

class Circle:
    def __init__(self, centre: Coordinate, radius: float):
        self.centre = centre
        self.radius = radius

    @staticmethod
    def intersection_between_circles(circle1, circle2):
        dx = circle2.centre.x - circle1.centre.x
        dy = circle2.centre.y - circle1.centre.y
        distance = np.sqrt(np.pow(dx, 2) + np.pow(dy, 2))

        intersection_coords = []

        if distance < circle1.radius + circle2.radius and distance > 0:
            a = (np.pow(circle1.radius, 2) - np.pow(circle2.radius, 2) + np.pow(distance, 2)) / (2 * distance)
            if (np.pow(circle1.radius, 2) - np.pow(a, 2)) < 0:
                raise ValueError("No intersections")
            h = np.sqrt(np.pow(circle1.radius, 2) - np.pow(a, 2))
            x5 = circle1.centre.x + a/distance * dx
            y5 = circle1.centre.y + a/distance * dy

            x3 = x5 - h/distance * dy
            y3 = y5 + h/distance * dx
            intersection_coords.append(Coordinate(x3, y3))

            x4 = x5 + h/distance * dy
            y4 = y5 - h/distance * dx
            intersection_coords.append(Coordinate(x4, y4))

            if len(intersection_coords) < 1:
                raise ValueError("No intersections found")

        return intersection_coords

class Line:
    def __init__(self, point1: Coordinate, point2: Coordinate):
        self.point1 = point1
        self.point2 = point2

    @property
    def length(self) -> float:
        dx = np.abs(self.point1.x - self.point2.x)
        dy = np.abs(self.point1.y - self.point2.y)
        return np.hypot(dx, dy)
    
    @property
    def midpoint(self) -> Coordinate:
        dx = (self.point2.x - self.point1.x)/2
        dy = (self.point2.y - self.point1.y)/2
        return Coordinate(self.point1.x + dx, self.point2.y + dy)

class Tension:
    def __init__(self, tension: float):
        self.tension_N = tension

    @classmethod
    def from_newtons(cls, tension: float):
        return cls(tension)

class Mass:
    def __init__(self, mass_kg: float):
        self.mass_kg = mass_kg
    
    @classmethod
    def from_kg(cls, mass_kg: float):
        return cls(mass_kg)

class Angle:
    def __init__ (self, angle_rad: float):
        self.rad = angle_rad

    @property
    def deg(self) -> float:
        return np.degrees(self.rad)
    
    @classmethod
    def from_degrees(cls, angle_deg: float) -> Angle:
        angle_rad = np.radians(angle_deg)
        return cls(angle_rad)

    @classmethod
    def from_radians(cls, angle_rad: float) -> Angle:
        return cls(angle_rad)

    @classmethod
    def down(cls):
        return cls(np.radians(-90))
    
    @staticmethod
    def inverse(angle: Angle) -> Angle:
         return Angle.from_radians((angle.rad + np.pi) % (2 * np.pi))

class ForceVector:
    def __init__(self, dir: Angle, mag: Tension):
        self.dir = dir
        self.mag = mag

    def __add__(self, other: ForceVector) -> ForceVector:
        return ForceVector.from_components(self.x_component + other.x_component, self.y_component + other.y_component)
    
    @classmethod
    def from_components(cls, x, y):
        dir = Angle.from_radians(np.arctan2(y, x))
        mag = Tension.from_newtons(np.hypot(x, y))
        return cls(dir, mag)
    
    @classmethod
    def gravity(cls, mass: Mass):
        dir = Angle.down()
        mag = Tension.from_newtons(mass.mass_kg * g)
        return cls(dir, mag)
    
    @classmethod
    def zero(cls) -> ForceVector:
        return cls(Angle.from_degrees(0), Tension.from_newtons(0))
    
    @property
    def x_component(self):
        return self.mag.tension_N * np.cos(self.dir.rad)
    
    @property
    def y_component(self):
        return self.mag.tension_N * np.sin(self.dir.rad)
    
    @property
    def magnitude(self):
        return Tension.from_newtons(np.hypot(self.x_component, self.y_component))

    @staticmethod
    def find_equilibrium_vector(vectors: list[ForceVector]) -> ForceVector:    
        sum = ForceVector.sum_forces(vectors)
        return ForceVector.from_components(-sum.x_component, -sum.y_component)
    
    @staticmethod
    def sum_forces(forces: list[ForceVector]) -> ForceVector:
        x_component_sum = 0
        y_component_sum = 0
        for force in forces:
            x_component_sum = x_component_sum + force.x_component
            y_component_sum = y_component_sum + force.y_component
    
        return ForceVector.from_components(x_component_sum, y_component_sum)
    
    @staticmethod
    def find_equilibirum_vectors(known_loads: list[ForceVector], unknown_load_directions: list[Angle]) -> list[ForceVector]:
        if len(unknown_load_directions) != 2:
            raise ValueError(f"{len(unknown_load_directions)} is not what this solver is designed for (2)")

        rx = 0
        ry = 0

        for load in known_loads:
            rx = rx + load.x_component
            ry = ry + load.y_component

        theta1 = unknown_load_directions[0].rad
        theta2 = unknown_load_directions[1].rad

        f1_mag = (rx - (ry/np.tan(theta2)))/(np.cos(theta1) - (np.sin(theta1)/np.tan(theta2)))
        f2_mag = (ry-f1_mag*np.sin(theta1))/np.sin(theta2)

        f1 = ForceVector(Angle.from_radians(theta1), Tension.from_newtons(f1_mag))
        f2 = ForceVector(Angle.from_radians(theta2), Tension.from_newtons(f2_mag))

        return [f1, f2]

        
    
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
    
    @staticmethod
    def find_vertical_reactions(known_loads: list[tuple[ForceVector, Coordinate]], reaction_centers: list[Coordinate]) -> list[tuple[ForceVector, Coordinate]]:
        if len(reaction_centers) > 2:
            raise ValueError(f"{len(reaction_centers)} is more than this solver is designed for (2)")
        
        sum_vertical_loads = 0
        sum_moments_about_reaction1 = 0

        for load in known_loads:
            sum_vertical_loads = sum_vertical_loads + load[0].y_component
            sum_moments_about_reaction1 = sum_moments_about_reaction1 + ((load[1].x - reaction_centers[0].x) * load[0].y_component)
        
        reaction2 = ForceVector.from_components(0, sum_moments_about_reaction1/(reaction_centers[0].x - reaction_centers[1].x))
        reaction1 = ForceVector.from_components(0, -(reaction2.y_component + sum_vertical_loads))

        return [(reaction1, reaction_centers[0]), (reaction2, reaction_centers[1])]


class Torque():
    def __init__(self, torque: float):
        self.torque_Nmm = torque

    @classmethod
    def from_Nmm(cls, torque: float) -> Torque:
        return cls(torque)
    
        





        



    


        
        

        
