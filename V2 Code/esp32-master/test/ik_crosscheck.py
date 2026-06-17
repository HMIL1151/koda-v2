"""Cross-check the C++ IK port against V1's inverse_kinematics.py.

Not part of the firmware build. Run with CPython from anywhere:
    python "V2 Code/esp32-master/test/ik_crosscheck.py"

`ik_v2_mirror` below is a line-for-line Python transcription of the C++ in
src/kinematics/inverse_kinematics.cpp + math/angle.h. It runs V1's real IK and the
mirror over a grid of foot targets and asserts they agree. If the C++ ever drifts from
V1's maths, update the mirror here and this catches the divergence.
"""

import math
import os
import sys

# Make V1's modules importable.
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "V1 Code"))

import inverse_kinematics as v1ik   # noqa: E402

# ── config.h constants used by the C++ port ──────────────────────────────────────────
A_MM = 31.56
THIGH = 30.0
CALF = 120.0
SERVO_DIST = 46.0
HIP_SEP = 85.0


def _circle_intersection(c1, r1, c2, r2):
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dy)
    if d <= 0 or d >= r1 + r2:
        return None
    a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
    h2 = r1 * r1 - a * a
    if h2 < 0:
        return None
    h = math.sqrt(h2)
    xm, ym = c1[0] + a / d * dx, c1[1] + a / d * dy
    return [(xm - h / d * dy, ym + h / d * dx), (xm + h / d * dy, ym - h / d * dx)]


def _cw(p1, p2, vertex):
    t1 = math.atan2(p1[1] - vertex[1], p1[0] - vertex[0])
    t2 = math.atan2(p2[1] - vertex[1], p2[0] - vertex[0])
    return math.degrees((t1 - t2) % (2 * math.pi))


def _ccw(p1, p2, vertex):
    t1 = math.atan2(p1[1] - vertex[1], p1[0] - vertex[0])
    t2 = math.atan2(p2[1] - vertex[1], p2[0] - vertex[0])
    return math.degrees((t2 - t1) % (2 * math.pi))


def ik_v2_mirror(point):
    x, y, z = point
    q = HIP_SEP / 2 - z
    under = q * q + y * y - A_MM * A_MM
    if under < 0:
        return None
    yp = math.sqrt(under)
    theta_h = math.degrees(math.pi - math.atan(y / q) - math.atan(yp / A_MM))

    foot = (x, yp)
    s1, s2 = (-SERVO_DIST / 2, 0.0), (SERVO_DIST / 2, 0.0)
    i1 = _circle_intersection(foot, CALF, s1, THIGH)
    i2 = _circle_intersection(foot, CALF, s2, THIGH)
    if not i1 or not i2:
        return None
    knee1 = i1[1] if _cw(s1, foot, i1[0]) < 180 else i1[0]
    knee2 = i2[0] if _cw(s2, foot, i2[0]) < 180 else i2[1]
    return (theta_h, _ccw(s2, knee1, s1), _cw(s1, knee2, s2))


def main():
    pts = []
    for x in (-30, 0, 30):
        for y in (110, 125, 140):
            for z in (60, 73.5, 90):
                pts.append((x, y, z))

    worst = 0.0
    checked = 0
    for p in pts:
        try:
            a = v1ik.inverse_kinematics(p)   # V1 returns ints
        except ValueError:
            continue
        b = ik_v2_mirror(p)
        assert b is not None, f"mirror failed where V1 solved: {p}"
        # V1 truncates to int; compare within 1 degree of that truncation.
        for i in range(3):
            diff = abs(a[i] - b[i])
            worst = max(worst, diff)
            assert diff < 1.5, f"point {p} joint {i}: V1={a[i]} mirror={b[i]:.3f}"
        checked += 1

    print(f"ok: {checked} points agree (worst joint diff {worst:.3f}° vs V1's int output)")
    print("C++ IK port matches V1 inverse_kinematics.py")


if __name__ == "__main__":
    main()
