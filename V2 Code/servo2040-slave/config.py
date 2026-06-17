"""Servo2040 slave configuration: servo map, calibration, limits, link + loop timing.

This is the one place physical, board-specific numbers live. The master thinks in clean
logical angles; everything that depends on how a servo is wired or mounted — direction,
trim, end-stops — is a row in CALIBRATION below.
"""

# Set True to run the slave logic on a PC against mock_lib (no Pimoroni hardware).
EMULATION_MODE = False

# ── UART link to the ESP32 master (see PROTOCOL.md) ──────────────────────────────────
# Use GPIO broken out on the Servo2040 and mapped to RP2040 UART1. Wire:
#   ESP32 TX → SLAVE_RX_PIN,  ESP32 RX ← SLAVE_TX_PIN,  common GND.
SLAVE_UART_ID = 1
SLAVE_TX_PIN  = 8     # RP2040 UART1 TX option
SLAVE_RX_PIN  = 9     # RP2040 UART1 RX option
SLAVE_BAUD    = 115200

# ── Loop / link timing ───────────────────────────────────────────────────────────────
LOOP_HZ          = 200      # slave runs faster than the master's 50 Hz to smooth motion
LINK_TIMEOUT_MS  = 200      # no valid frame in this window → fault + fail-safe
STATUS_PERIOD_MS = 100      # how often to send a STATUS frame back to the master

# ── Motion limits ────────────────────────────────────────────────────────────────────
MAX_SLEW_DEG_PER_S = 600.0  # cap servo speed; the slave eases targets at this rate

NUM_SERVOS = 12

# Logical channel order — MUST match the master's cfg::ServoChannel enum.
(FL_HIP, FL_KNEE_L, FL_KNEE_R,
 FR_HIP, FR_KNEE_L, FR_KNEE_R,
 RR_HIP, RR_KNEE_L, RR_KNEE_R,
 RL_HIP, RL_KNEE_L, RL_KNEE_R) = range(NUM_SERVOS)

# Servo2040 hardware pins per channel (same wiring as V1 robot.py).
# Resolved lazily in servo_controller so this module imports fine under emulation.
SERVO_PIN_NAMES = [
    "SERVO_1", "SERVO_7", "SERVO_13",   # FL: hip, knee-L, knee-R
    "SERVO_2", "SERVO_8", "SERVO_14",   # FR
    "SERVO_3", "SERVO_9", "SERVO_15",   # RR
    "SERVO_4", "SERVO_10", "SERVO_16",  # RL
]

# Per-servo calibration. The master sends a *logical* angle (deg); the slave outputs
#   value = direction * logical + trim,  clamped to [min, max].
# direction/trim absorb mounting; min/max are mechanical end-stops (safety clamp).
# `trim` for RL_HIP carries V1's old +10° left-rear hip fudge — now lives here, not in
# the kinematics.
class Cal:
    __slots__ = ("direction", "trim", "min", "max")
    def __init__(self, direction=1, trim=0.0, lo=-360.0, hi=360.0):
        self.direction = direction
        self.trim = trim
        self.min = lo
        self.max = hi

CALIBRATION = [Cal() for _ in range(NUM_SERVOS)]
CALIBRATION[RL_HIP].trim = 10.0     # ported from V1 leg.py left-rear hip offset

# Safe / crouch pose the slave eases to on link loss or RELAX→SAFE, in *logical* angles.
# Chosen to match V1's hips-up crouch (HIP_UP_ANGLE_DEG, 180, 180).
SAFE_POSE_DEG = [90.0, 180.0, 180.0,
                 90.0, 180.0, 180.0,
                 90.0, 180.0, 180.0,
                 90.0, 180.0, 180.0]

# ── Status LEDs ──────────────────────────────────────────────────────────────────────
NUM_LEDS = 6
