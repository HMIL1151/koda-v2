"""The servo HAL: 12 servos with per-channel calibration, speed-limited smoothing and a
mechanical-limit clamp.

The master streams logical target angles at ~50 Hz; this controller runs faster and eases
each servo toward its target at MAX_SLEW_DEG_PER_S, so motion stays smooth even if a
packet is late. Every output is clamped to the servo's calibrated end-stops before it
reaches hardware — the slave never drives a servo past a safe angle, whatever the master
asked for.
"""

import config

if config.EMULATION_MODE:
    from mock_lib.servo import Servo, servo2040
else:
    from servo import Servo, servo2040


def _clamp(value, lo, hi):
    return lo if value < lo else (hi if value > hi else value)


class ServoController:
    def __init__(self):
        self._servos = [Servo(getattr(servo2040, name)) for name in config.SERVO_PIN_NAMES]
        self._target = list(config.SAFE_POSE_DEG)    # logical target angles (deg)
        self._current = list(config.SAFE_POSE_DEG)   # logical, smoothed
        self._clamped = [False] * config.NUM_SERVOS
        self._out_of_range = 0
        self.enabled = False

    # ── Torque control ────────────────────────────────────────────────────────────────
    def enable(self):
        for servo in self._servos:
            servo.enable()
        self.enabled = True

    def disable(self):
        for servo in self._servos:
            servo.disable()
        self.enabled = False

    # ── Target setting ────────────────────────────────────────────────────────────────
    def set_targets(self, logical_angles):
        """Update targets from a decoded SERVO_TARGETS list; None entries are held."""
        for i, angle in enumerate(logical_angles):
            if angle is not None:
                self._target[i] = angle

    def set_pose(self, logical_angles):
        """Set every target at once (used for the safe pose)."""
        for i, angle in enumerate(logical_angles):
            self._target[i] = angle

    def at_pose(self, logical_angles, tol=1.0):
        """True once the smoothed position has reached `logical_angles` within tol deg."""
        return all(abs(self._current[i] - logical_angles[i]) <= tol
                   for i in range(config.NUM_SERVOS))

    # ── Per-tick update ───────────────────────────────────────────────────────────────
    def update(self, dt):
        """Ease toward targets and write calibrated, clamped values to the servos."""
        if not self.enabled:
            return
        max_step = config.MAX_SLEW_DEG_PER_S * dt
        for i, servo in enumerate(self._servos):
            delta = _clamp(self._target[i] - self._current[i], -max_step, max_step)
            self._current[i] += delta

            cal = config.CALIBRATION[i]
            value = cal.direction * self._current[i] + cal.trim
            clamped = value < cal.min or value > cal.max
            if clamped:
                self._out_of_range += 1
            self._clamped[i] = clamped
            servo.value(_clamp(value, cal.min, cal.max))

    # ── Status ────────────────────────────────────────────────────────────────────────
    @property
    def any_clamped(self):
        return any(self._clamped)

    @property
    def out_of_range_count(self):
        return self._out_of_range
