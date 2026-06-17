// Koda V2 master — central configuration: pins, robot geometry, tunables.
// All robot-physical constants from V1's constants.py live here, ported 1:1 unless
// noted. Keep numbers here, not scattered through the code.
#pragma once

#include <cstdint>

namespace cfg {

// ─────────────────────────────────────────────────────────────────────────────
//  Control loop
// ─────────────────────────────────────────────────────────────────────────────
constexpr float CONTROL_HZ = 50.0f;                 // master tick rate (V1 SERVO_FREQUENCY)
constexpr uint32_t CONTROL_PERIOD_US =
    static_cast<uint32_t>(1'000'000.0f / CONTROL_HZ);

// ─────────────────────────────────────────────────────────────────────────────
//  UART link to the Servo2040 slave  (see PROTOCOL.md)
// ─────────────────────────────────────────────────────────────────────────────
constexpr int      LINK_UART_NUM = 1;               // ESP32 UART1
constexpr uint32_t LINK_BAUD     = 115200;
constexpr int      LINK_TX_PIN   = 17;              // ESP32 TX → 2040 RX
constexpr int      LINK_RX_PIN   = 16;              // ESP32 RX ← 2040 TX
constexpr uint32_t STATUS_REQUEST_HZ = 10;          // how often we expect slave status

// ─────────────────────────────────────────────────────────────────────────────
//  Leg sign conventions  (V1: LEFT=-1, RIGHT=1, FRONT=1, REAR=-1)
// ─────────────────────────────────────────────────────────────────────────────
enum Side { LEFT = -1, RIGHT = 1 };
enum Face { REAR = -1, FRONT = 1 };

// ─────────────────────────────────────────────────────────────────────────────
//  Leg geometry (mm / deg) — ported from V1 constants.py
// ─────────────────────────────────────────────────────────────────────────────
constexpr float HIP_SEPARATION_MM = 85.0f;          // V1 HIP_SEPERATION_MM
constexpr float SERVO_DISTANCE_MM = 46.0f;          // distance between the 2 knee-driver servos
constexpr float THIGH_LENGTH_MM   = 30.0f;
constexpr float CALF_LENGTH_MM    = 120.0f;

// NOTE: V1's inverse_kinematics.py declared LEG_GEOMETRY_A_MM = 80 then immediately
// overwrote it with `a = 31.56`. The 31.56 value is what actually drove the robot, so
// that is the real geometry constant. Preserved here under a clear name.
constexpr float LEG_GEOMETRY_A_MM = 31.56f;         // hip-pivot offset used by hip-angle solve

// Neutral foot target in leg frame (V1 ZERO_X / ZERO_Y / ZERO_Z).
constexpr float ZERO_X = 0.0f;
constexpr float ZERO_Y = 125.0f;
constexpr float ZERO_Z = 85.0f / 2.0f + 31.0f;      // = 73.5

// Servo-convention mapping (V1 leg.py)
constexpr float HIP_UP_ANGLE_DEG = 90.0f;
constexpr float SERVO_OFFSET_DEG = 180.0f;

// ─────────────────────────────────────────────────────────────────────────────
//  Body geometry (V1 orientation.py) — for pose → foot-target maths
// ─────────────────────────────────────────────────────────────────────────────
constexpr float LEG_X_SEPARATION_MM = 221.0f;       // front↔rear hip spacing
constexpr float TORSO_WIDTH_Y_MM    = 85.0f;
constexpr float TORSO_HEIGHT_Z_MM   = 48.0f;

// ─────────────────────────────────────────────────────────────────────────────
//  Gait tunables (V1 gait.py)
// ─────────────────────────────────────────────────────────────────────────────
constexpr float STEP_HEIGHT_MM        = 40.0f;
constexpr float STEP_CURVE_DELTA       = 0.2f;       // bezier overshoot fraction of step len
constexpr float MANOUVRE_STEP_DIST_MM = 15.0f;       // turn / strafe step length
constexpr float MANOUVRE_SPEED_MM_S   = 50.0f;
constexpr float WALK_STEP_DIST_MM     = 45.0f;
constexpr float WALK_SPEED_MM_S       = 200.0f;
constexpr int   SWING_SAMPLES         = 24;          // bezier resolution for one swing

// ─────────────────────────────────────────────────────────────────────────────
//  Hall-effect force sensors — see sensors/hall_sensor.h + sensors/hall_calibrator.h
//
//  Two calf springs per foot, one hall sensor each → 8 sensors. Per-foot force is the
//  sum of its two calf sensors. They hang off two external ADS1115 I2C ADCs because the
//  ESP32's own ADC2 is unusable while Bluetooth (Bluepad32) is active and ADC1 hasn't
//  enough free channels for 8.
//
//  Physical model (from the bench calibration sketch): signal = K / (distance + offset)^3
//  The offset absorbs finite magnet size, Hall-IC depth and non-point field behaviour.
// ─────────────────────────────────────────────────────────────────────────────
constexpr int NUM_FEET         = 4;
constexpr int SENSORS_PER_FOOT = 2;                  // two calves per foot
constexpr int NUM_HALL_SENSORS = NUM_FEET * SENSORS_PER_FOOT;  // = 8

// External ADC: two ADS1115 (4 channels each) on I2C. Sensor index → (ads index, channel).
constexpr uint8_t ADS1115_ADDR[2] = {0x48, 0x49};    // ADDR→GND, ADDR→VDD
constexpr int     HALL_I2C_SDA = 21;
constexpr int     HALL_I2C_SCL = 22;
constexpr int     HALL_ADC_MAX = 32767;              // ADS1115 single-ended full scale

// Read a few sensors per tick (round-robin) so one control tick never blocks on 8 slow
// ADC conversions; the EMA smooths the rest. Calibration capture reads all, carefully.
constexpr int   HALL_READS_PER_TICK   = 2;
constexpr int   HALL_FILTER_SAMPLES   = 15;          // averaged samples for a careful read
constexpr int   HALL_SAMPLE_DELAY_US  = 300;         // settle between calibration samples
constexpr float HALL_EMA_ALPHA        = 0.30f;       // 0..1 smoothing on per-sensor force

// Inverse-cube magnetic model constants (ported from the bench sketch).
constexpr float HALL_ZERO_LOAD_DIST_MM = 17.0f;      // spring length, unloaded
constexpr float HALL_FULL_LOAD_DIST_MM = 1.0f;       // EFFECTIVE magnetic distance at full load (NOT 0)
constexpr float HALL_FULL_LOAD_FORCE_N = 27.05f;     // known spring force at full compression
constexpr float HALL_MAGNET_OFFSET_MM  = 5.0f;       // magnetic geometry compensation — tune
constexpr float HALL_SPRING_N_PER_MM =
    HALL_FULL_LOAD_FORCE_N / (HALL_ZERO_LOAD_DIST_MM - HALL_FULL_LOAD_DIST_MM);

// Persisted calibration (LittleFS JSON). Loaded at boot; rewritten after a calibration.
constexpr const char* HALL_CAL_PATH    = "/hall_cal.json";
constexpr int         HALL_CAL_VERSION = 1;

// ─────────────────────────────────────────────────────────────────────────────
//  Early ground-contact detection (sensors/ground_contact)
// ─────────────────────────────────────────────────────────────────────────────
constexpr float CONTACT_FORCE_N       = 4.0f;        // force above which a foot is "on the ground"
constexpr float CONTACT_RELEASE_N     = 2.0f;        // hysteresis: below this = airborne
constexpr float EARLY_CONTACT_MIN_PHASE = 0.35f;     // only trust early contact past this swing fraction

// ─────────────────────────────────────────────────────────────────────────────
//  Balance / COG management on an incline (control/balance)
// ─────────────────────────────────────────────────────────────────────────────
constexpr float BALANCE_KP_PITCH   = 0.6f;           // body pitch correction per rad slope
constexpr float BALANCE_KP_ROLL    = 0.6f;
constexpr float BALANCE_KP_SHIFT   = 0.8f;           // COG shift gain (mm per N of imbalance)
constexpr float BALANCE_MAX_SHIFT_MM   = 30.0f;      // clamp on COG translation
constexpr float BALANCE_MAX_TILT_DEG   = 12.0f;      // clamp on torso pitch/roll correction
constexpr bool  HAS_IMU            = false;          // set true once an IMU is wired (sensors/imu)

// ─────────────────────────────────────────────────────────────────────────────
//  Servo channel order on the wire (must mirror config.py:ServoChannel on the slave)
// ─────────────────────────────────────────────────────────────────────────────
enum ServoChannel : uint8_t {
  FL_HIP = 0, FL_KNEE_L, FL_KNEE_R,
  FR_HIP,     FR_KNEE_L, FR_KNEE_R,
  RR_HIP,     RR_KNEE_L, RR_KNEE_R,
  RL_HIP,     RL_KNEE_L, RL_KNEE_R,
  NUM_SERVOS
};

}  // namespace cfg
