from math import factorial
from constants import ZERO_X, ZERO_Y, ZERO_Z
from units import Direction

def binomial_coefficient(n, k):
    return factorial(n) // (factorial(k) * factorial(n - k))

def calculate_curve(step_length, step_height, delta, stance_steps, swing_steps, direction):

    L = step_length
    H = step_height
    d = delta * L

    if direction == Direction.FORWARDS or direction == Direction.BACKWARDS:

        P0 = (ZERO_X - L/2,     ZERO_Y,     ZERO_Z)
        P1 = (ZERO_X - L/2 - d, ZERO_Y,     ZERO_Z)
        P2 = (ZERO_X - L/2,     ZERO_Y - H, ZERO_Z)
        P3 = (ZERO_X + L/2,     ZERO_Y - H, ZERO_Z)
        P4 = (ZERO_X + L/2 + d, ZERO_Y,     ZERO_Z)
        P5 = (ZERO_X + L/2,     ZERO_Y,     ZERO_Z)

        points = [P0, P1, P2, P3, P4, P5]

        if direction == Direction.BACKWARDS:
            points = points[::-1]

    if direction == Direction.LEFT or direction == Direction.RIGHT:

        P0 = (ZERO_X,     ZERO_Y,     ZERO_Z - L/2)
        P1 = (ZERO_X,     ZERO_Y,     ZERO_Z - L/2 - d)
        P2 = (ZERO_X,     ZERO_Y - H, ZERO_Z - L/2)
        P3 = (ZERO_X,     ZERO_Y - H, ZERO_Z + L/2)
        P4 = (ZERO_X,     ZERO_Y,     ZERO_Z + L/2 + d)
        P5 = (ZERO_X,     ZERO_Y,     ZERO_Z + L/2)

        points = [P0, P1, P2, P3, P4, P5]

        if direction == Direction.RIGHT:
            points = points[::-1]


    n = len(points) - 1
    path_points = []
    for j in range(swing_steps):
        t = j / (swing_steps - 1) if swing_steps > 1 else 0
        x = 0.0
        z = 0.0
        y = 0.0
        for i, P in enumerate(points):
            binom = binomial_coefficient(n, i)
            basis = binom * (1 - t) ** (n - i) * t ** i
            x += basis * P[0]
            y += basis * P[1]
            z += basis * P[2]
        path_points.append((x, y, z))

    x0, y0, z0 = points[-1]
    x1, y1, z1 = points[0]

    for k in range(1, stance_steps + 1):
        t = k / stance_steps
        x = x0 + (x1 - x0) * t
        z = z0 + (z1 - z0) * t
        y = y0 + (y1 - y0) * t
        path_points.append((x, y, z))

    return path_points
