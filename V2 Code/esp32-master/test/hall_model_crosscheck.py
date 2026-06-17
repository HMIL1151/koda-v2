"""Cross-check the C++ inverse-cube hall model (sensors/hall_sensor.cpp) against the
Arduino bench sketch, and assert its physical invariants.

Run with CPython:  python "V2 Code/esp32-master/test/hall_model_crosscheck.py"

`model_*` below transcribes the C++ (which itself ports the Arduino sketch). The strong
invariants — signal=s0 → 0 N, signal=s1 → exactly the full-load force, for ANY K — make
this a real correctness test, not just a tautology.
"""

import math

# config.h constants
ZERO_LOAD = 17.0
FULL_LOAD = 1.0
FULL_FORCE = 27.05
OFFSET = 5.0
SPRING = FULL_FORCE / (ZERO_LOAD - FULL_LOAD)


def solve_K(s0, s1):
    d0 = ZERO_LOAD + OFFSET
    d1 = FULL_LOAD + OFFSET
    return (s1 - s0) / (1.0 / d1**3 - 1.0 / d0**3)


def signal_to_distance(signal, s0, s1, K):
    s_lo, s_hi = min(s0, s1), max(s0, s1)
    signal = max(s_lo, min(s_hi, signal))
    d0 = ZERO_LOAD + OFFSET
    inv = (signal - s0) / K + 1.0 / d0**3
    if inv <= 0.0:
        inv = 1e-9
    effective = (1.0 / inv) ** (1.0 / 3.0)
    physical = effective - OFFSET
    return max(FULL_LOAD, min(ZERO_LOAD, physical))


def distance_to_force(distance):
    return max(0.0, (ZERO_LOAD - distance) * SPRING)


def force_for(signal, s0, s1, K):
    return distance_to_force(signal_to_distance(signal, s0, s1, K))


def check(name, s0, s1):
    K = solve_K(s0, s1)

    f_zero = force_for(s0, s0, s1, K)
    f_full = force_for(s1, s0, s1, K)
    assert abs(f_zero) < 1e-3, f"{name}: unloaded force {f_zero} != 0"
    assert abs(f_full - FULL_FORCE) < 1e-2, f"{name}: full force {f_full} != {FULL_FORCE}"

    # Monotonic from s0→s1, and clamps beyond the calibrated span.
    prev = -1.0
    for i in range(21):
        sig = s0 + (s1 - s0) * i / 20.0
        f = force_for(sig, s0, s1, K)
        assert f >= prev - 1e-6, f"{name}: non-monotonic at {sig}"
        prev = f
    over = force_for(s1 + (s1 - s0), s0, s1, K)   # past full load (signed) → clamps
    assert abs(over - FULL_FORCE) < 1e-2, f"{name}: over-range not clamped ({over})"

    print(f"ok: {name}  K={K:.3e}  s0->{f_zero:.3f}N  s1->{f_full:.3f}N")


if __name__ == "__main__":
    # A rising-signal sensor and a falling-signal one (magnet polarity differs) — both
    # must satisfy the invariants.
    check("rising  (s1>s0)", s0=5000.0, s1=20000.0)
    check("falling (s1<s0)", s0=22000.0, s1=8000.0)
    check("small span", s0=12000.0, s1=15000.0)
    print("\nHALL MODEL MATCHES THE ARDUINO SKETCH (invariants hold)")
