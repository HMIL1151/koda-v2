import numpy as np
from units import *

def intersection_between_circles(circle1, circle2):
    # Unpack circle1 and circle2
    


    # Calculate the distance between the centers
    dx = circle2.centre.x - circle1.centre.x
    dy = circle2.centre.y - circle1.centre.y
    distance = np.sqrt(np.pow(dx, 2) + np.pow(dy, 2))

    intersection_coords = []

    # Check if circles intersect
    if distance < circle1.radius + circle2.radius and distance > 0:
        # Calculate intersection points (simplified)

        a = (np.pow(circle1.radius, 2) - np.pow(circle2.radius, 2) + np.pow(distance, 2)) / (2 * distance)
        if (np.pow(circle1.radius, 2) - np.pow(a, 2)) < 0:
            return intersection_coords
        h = np.sqrt(np.pow(circle1.radius, 2) - np.pow(a, 2))
        x5 = circle1.centre.x + a/distance * dx
        y5 = circle1.centre.y + a/distance * dy

        x3 = x5 - h/distance * dy
        y3 = y5 + h/distance * dx
        intersection_coords.append(Coordinate(x3, y3))

        x4 = x5 + h/distance * dy
        y4 = y5 - h/distance * dx
        intersection_coords.append(Coordinate(x4, y4))

    return intersection_coords