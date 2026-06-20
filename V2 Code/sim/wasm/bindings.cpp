// Emscripten/embind bindings — the bridge that lets the browser sim and node tests drive
// the REAL C++ control core. Everything here just adapts the firmware's own classes; no
// control logic is reimplemented. The same .cpp files compiled below also build for the
// ESP32, so the sim can never test stale logic.
//
// Build: see build.sh (compiles the firmware control core + this file to koda-core.mjs).
// Arrays cross the boundary as plain JS arrays (emscripten::val), so the JS side needs no
// special vector types.

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include "config.h"
#include "control/command.h"
#include "control/ground_contact.h"
#include "control/robot.h"
#include "control/slope_estimator.h"
#include "kinematics/inverse_kinematics.h"
#include "sensors/hall_model.h"
#include "sensors/imu.h"

using namespace emscripten;
using namespace koda;

namespace {

val to_js_array(const float* p, int n) {
  val a = val::array();
  for (int i = 0; i < n; ++i) a.call<void>("push", p[i]);
  return a;
}

// Button bitmask passed from JS (keeps the step() arg count sane).
enum Buttons : int { BTN_STAND = 1, BTN_SIT = 2, BTN_GAIT = 4 };

// One closed-loop control instance. Mirrors main.cpp's per-tick order exactly:
// ground-contact is updated from the PREVIOUS tick's leg phase (the same one-tick lag the
// firmware has), then the robot steps and emits 12 servo angles.
class SimCore {
 public:
  SimCore() { robot_.begin(); }

  // f0..f3 = per-foot ground reaction forces (N), order FL, FR, RR, RL.
  // Returns a JS array of the 12 logical servo angles (degrees), order cfg::ServoChannel.
  val step(float vx, float vy, float yaw, float height, int buttons,
           float f0, float f1, float f2, float f3, float pitch, float roll, float dt) {
    float f[cfg::NUM_FEET] = {f0, f1, f2, f3};

    contact_.update(f, robot_.local_phase(), robot_.swing_fraction());

    Command cmd;
    cmd.vx = vx;
    cmd.vy = vy;
    cmd.yaw = yaw;
    cmd.height = height;
    cmd.stand_toggle = buttons & BTN_STAND;
    cmd.sit_toggle = buttons & BTN_SIT;
    cmd.gait_cycle = buttons & BTN_GAIT;
    cmd.connected = true;

    Tilt tilt;
    tilt.pitch_rad = pitch;
    tilt.roll_rad = roll;

    float out[cfg::NUM_SERVOS];
    robot_.update(cmd, f, tilt, contact_, dt, out);
    return to_js_array(out, cfg::NUM_SERVOS);
  }

  // ── State, for visualization / assertions ──────────────────────────────────────────
  int state() const { return static_cast<int>(robot_.state()); }
  float cyclePhase() const { return robot_.cycle_phase(); }
  float swingFraction() const { return robot_.swing_fraction(); }
  // Live slope estimate (this tick's springs) and the latched MEASURING-window result.
  float slopePitch() const { return robot_.live_slope().pitch_rad; }
  float slopeRoll() const { return robot_.live_slope().roll_rad; }
  float slopeConfidence() const { return robot_.live_slope().confidence; }
  float measuredPitch() const { return robot_.measured_pitch(); }
  float measuredRoll() const { return robot_.measured_roll(); }
  // Latch a slope (drives the WALK feed-forward COG bias). MEASURING does this on the real
  // robot; exposed here for the sim/tests.
  void setMeasuredSlope(float pitch, float roll) { robot_.set_measured_slope(pitch, roll); }
  int terrain() const { return robot_.terrain(); }          // 0 flat, 1 uneven, 2 slope
  void setAutoSlope(bool on) { robot_.set_auto_slope(on); }
  bool autoSlope() const { return robot_.auto_slope(); }
  void setSlopeFollow(float gain) { robot_.set_slope_follow(gain); }
  val localPhase() const { return to_js_array(robot_.local_phase(), cfg::NUM_FEET); }
  bool earlyContact(int leg) const { return contact_.early_contact(leg); }
  bool lateContact(int leg) const { return contact_.late_contact(leg); }
  bool inContact(int leg) const { return contact_.in_contact(leg); }
  bool ikOk(int leg) const { return robot_.ik_ok(leg); }

  // ── Live step-geometry tuning ───────────────────────────────────────────────────────
  void setGaitParams(float stanceX, float stanceY, float stanceZ, float stepHeight,
                     float stepLen) {
    GaitParams p;
    p.stance_x = stanceX;
    p.stance_y = stanceY;
    p.stance_z = stanceZ;
    p.step_height = stepHeight;
    p.step_len = stepLen;
    robot_.set_gait_params(p);
  }
  // Current params as [stanceX, stanceY, stanceZ, stepHeight, stepLen] (for the UI defaults).
  val gaitParams() const {
    const GaitParams& p = robot_.gait_params();
    const float r[5] = {p.stance_x, p.stance_y, p.stance_z, p.step_height, p.step_len};
    return to_js_array(r, 5);
  }

  // This tick's foot target for a leg (leg frame, mm): [x fwd, y down, z right].
  val footTarget(int leg) const {
    const Vec3& f = robot_.foot_target(leg);
    const float r[3] = {f.x, f.y, f.z};
    return to_js_array(r, 3);
  }

 private:
  Robot robot_;
  GroundContact contact_;
};

// ── Free functions: the pure hall model + IK, for direct unit testing ─────────────────

// Inverse-cube hall model: raw signal + 2-point calibration → force (N).
float hallForceFromSignal(float signal, float s0, float s1, float K) {
  HallCal c;
  c.s0 = s0;
  c.s1 = s1;
  c.K = K;
  c.calibrated = true;
  return hall_model::force_from_signal(signal, c);
}

float solveHallK(float s0, float s1) {
  HallCal c;
  c.s0 = s0;
  c.s1 = s1;
  hall_model::solve_k(c);
  return c.K;
}

// Slope estimate from synthetic forces on a symmetric ±halfX/±halfZ footprint (for unit
// tests). Returns [pitchRad, rollRad, confidence].
val estimateSlope(float f0, float f1, float f2, float f3, float halfX, float halfZ,
                  float cogHeight) {
  const float forces[4] = {f0, f1, f2, f3};
  const Vec2 foot[4] = {                    // FL, FR, RR, RL  (x fwd, y right)
      {halfX, -halfZ}, {halfX, halfZ}, {-halfX, halfZ}, {-halfX, -halfZ}};
  const SlopeEstimate s = estimate_slope(forces, foot, cogHeight);
  const float r[3] = {s.pitch_rad, s.roll_rad, s.confidence};
  return to_js_array(r, 3);
}

// Inverse kinematics for one foot target. Returns [ok(1/0), hip, left, right] (degrees).
val inverseKinematics(float x, float y, float z) {
  JointAngles a;
  const bool ok = inverse_kinematics(Vec3{x, y, z}, a);
  const float r[4] = {ok ? 1.0f : 0.0f, a.hip, a.left, a.right};
  return to_js_array(r, 4);
}

}  // namespace

EMSCRIPTEN_BINDINGS(koda_core) {
  class_<SimCore>("SimCore")
      .constructor<>()
      .function("step", &SimCore::step)
      .function("state", &SimCore::state)
      .function("cyclePhase", &SimCore::cyclePhase)
      .function("swingFraction", &SimCore::swingFraction)
      .function("localPhase", &SimCore::localPhase)
      .function("earlyContact", &SimCore::earlyContact)
      .function("lateContact", &SimCore::lateContact)
      .function("inContact", &SimCore::inContact)
      .function("footTarget", &SimCore::footTarget)
      .function("ikOk", &SimCore::ikOk)
      .function("setGaitParams", &SimCore::setGaitParams)
      .function("gaitParams", &SimCore::gaitParams)
      .function("slopePitch", &SimCore::slopePitch)
      .function("slopeRoll", &SimCore::slopeRoll)
      .function("slopeConfidence", &SimCore::slopeConfidence)
      .function("measuredPitch", &SimCore::measuredPitch)
      .function("measuredRoll", &SimCore::measuredRoll)
      .function("setMeasuredSlope", &SimCore::setMeasuredSlope)
      .function("terrain", &SimCore::terrain)
      .function("setAutoSlope", &SimCore::setAutoSlope)
      .function("autoSlope", &SimCore::autoSlope)
      .function("setSlopeFollow", &SimCore::setSlopeFollow);

  function("hallForceFromSignal", &hallForceFromSignal);
  function("solveHallK", &solveHallK);
  function("inverseKinematics", &inverseKinematics);
  function("estimateSlope", &estimateSlope);

  // Expose a few constants the sim/tests need so they can't drift from config.h.
  constant("NUM_FEET", (int)cfg::NUM_FEET);
  constant("NUM_SERVOS", (int)cfg::NUM_SERVOS);
  constant("ZERO_X", (float)cfg::ZERO_X);
  constant("ZERO_Y", (float)cfg::ZERO_Y);
  constant("ZERO_Z", (float)cfg::ZERO_Z);
  constant("LEG_X_SEPARATION_MM", (float)cfg::LEG_X_SEPARATION_MM);
  constant("SERVO_DISTANCE_MM", (float)cfg::SERVO_DISTANCE_MM);
  constant("THIGH_LENGTH_MM", (float)cfg::THIGH_LENGTH_MM);
  constant("CALF_LENGTH_MM", (float)cfg::CALF_LENGTH_MM);
  constant("HALL_FULL_LOAD_FORCE_N", (float)cfg::HALL_FULL_LOAD_FORCE_N);
  constant("HALL_SPRING_N_PER_MM", (float)cfg::HALL_SPRING_N_PER_MM);
}
