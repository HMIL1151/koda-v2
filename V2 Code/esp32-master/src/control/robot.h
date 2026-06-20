// The robot: four legs, a gait, a balance controller and a small state machine. This is
// the V2 replacement for V1's 500-line robot.py — the blocking pose sequences are gone,
// replaced by a non-blocking per-tick update() that emits 12 servo angles.
#pragma once

#include "config.h"
#include "control/balance.h"
#include "control/body_pose.h"
#include "control/command.h"
#include "control/ground_contact.h"
#include "control/leg.h"
#include "control/slope_estimator.h"
#include "control/terrain_monitor.h"
#include "gait/gait.h"
#include "protocol/protocol.h"
#include "sensors/imu.h"

namespace koda {

// STOPPING is the graceful walk→stand wind-down; MEASURING holds a static reference stance
// to read the slope from the springs (the static part of the auto stop-measure-resume cycle).
enum class RobotState { SLEEP, STAND, WALK, SIT, STOPPING, MEASURING };

// Runtime-tunable step geometry (defaults from config.h). Settable live — handy for tuning
// the gait in the simulator or over the link without recompiling.
struct GaitParams {
  float stance_x = cfg::ZERO_X;            // neutral foot fore/aft
  float stance_y = cfg::ZERO_Y;            // neutral foot drop (stance height)
  float stance_z = cfg::ZERO_Z;            // neutral foot lateral offset (stance width / track)
  float step_height = cfg::STEP_HEIGHT_MM;  // swing lift
  float step_len = cfg::WALK_STEP_DIST_MM;  // forward stride length
  Vec3 stance() const { return {stance_x, stance_y, stance_z}; }
};

class Robot {
 public:
  Robot();

  void begin();

  // One control tick.
  //   cmd     : operator command (already normalised/debounced)
  //   forces  : per-foot ground reaction force (N), order FL, FR, RR, RL
  //   tilt    : torso tilt from the IMU (level if none)
  //   contact : early/!contact flags, already updated for this tick
  //   dt      : seconds since the last tick
  //   out     : receives the 12 logical servo angles (degrees); NaN = hold channel
  void update(const Command& cmd, const float* forces, Tilt tilt,
              const GroundContact& contact, float dt,
              float out[cfg::NUM_SERVOS]);

  RobotState state() const { return state_; }
  proto::SlaveMode desired_slave_mode() const;
  float cycle_phase() const { return phase_; }
  const float* local_phase() const { return local_phase_; }
  float swing_fraction() const { return gait_.timing().swing_fraction; }
  // The foot target this tick produced, in the leg frame (x fwd, y down, z right).
  // Useful for state estimation and for the off-hardware simulator.
  const Vec3& foot_target(int leg) const { return foot_target_[leg]; }
  // Slope estimated from the calf springs this tick (only meaningful from a static,
  // unbalanced reference stance — see SlopeEstimator). The MEASURING state latches it.
  const SlopeEstimate& live_slope() const { return slope_est_; }
  float measured_pitch() const { return measured_slope_.pitch_rad; }
  float measured_roll() const { return measured_slope_.roll_rad; }
  float measured_confidence() const { return measured_slope_.confidence; }
  // Latch a slope measurement: drives the WALK feed-forward COG bias. Called by the
  // MEASURING state; also a test/telemetry hook.
  void set_measured_slope(float pitch_rad, float roll_rad) {
    measured_slope_.pitch_rad = pitch_rad;
    measured_slope_.roll_rad = roll_rad;
    balance_.set_slope(pitch_rad, roll_rad);
  }
  // Did this leg's IK solve this tick? False = the foot target is unreachable / out of
  // range (the controller held the last good pose for it).
  bool ik_ok(int leg) const { return ik_ok_[leg]; }

  // Live step-geometry tuning.
  void set_gait_params(const GaitParams& p) { params_ = p; }
  const GaitParams& gait_params() const { return params_; }

  // Automatic slope handling (stop → measure → adjust → resume). On by default.
  void set_auto_slope(bool on) { auto_slope_ = on; }
  bool auto_slope() const { return auto_slope_; }
  void set_slope_follow(float gain) { slope_follow_gain_ = gain; }   // gait plane-follow gain
  int terrain() const { return static_cast<int>(terrain_.classify()); }  // 0 flat,1 uneven,2 slope

 private:
  // Leg position in the body frame (x fwd, z right) — lever arm for posture + turn centre.
  // Lateral half-span tracks the tunable stance width.
  Vec2 leg_body_pos(int leg) const;
  // Apply a body pose (translation + pitch/roll lever-arm) to a foot in the leg frame.
  Vec3 apply_body_pose(Vec3 foot, int leg, const BodyPose& pose) const;
  // Vertical offset that places a leg's foot on the measured ground plane (gait pre-tilt).
  float slope_follow(int leg) const;
  // Map a body pose + height trim onto a leg's neutral foot target (leg frame).
  Vec3 neutral_foot(int leg, const BodyPose& pose, float height_trim) const;
  // One leg's gait foot target for a given phase, with separate stride and lift scaling
  // (height_scale eases the swing lift in/out), and early-contact plant.
  Vec3 gait_foot(int leg, const Command& cmd, float stride_scale, float height_scale,
                 const GroundContact& contact) const;
  // Extra downward reach for late contact: while a stance foot feels no ground it reaches
  // down to find it (reset each swing). Returns the current probe offset (mm), mutating it.
  float ground_probe(int leg, float dt, const GroundContact& contact);
  // Per-leg ground-plane step vector for the current command.
  Vec2 step_vector(int leg, const Command& cmd) const;
  // Solve all four legs into `out`, holding last-good angles on any IK miss.
  void solve_all(const Vec3 feet[cfg::NUM_FEET], float out[cfg::NUM_SERVOS]);
  void cycle_gait();

  Leg legs_[cfg::NUM_FEET];
  Gait gait_;
  Balance balance_;
  TerrainMonitor terrain_;
  GaitParams params_;
  float slope_follow_gain_ = cfg::GAIT_SLOPE_FOLLOW;
  // Auto stop-measure-resume cycle.
  bool  auto_slope_ = true;
  bool  measure_pending_ = false;   // a measure cycle has been triggered
  float measure_timer_ = 0.0f;      // settle time spent in MEASURING
  float measure_cooldown_ = 0.0f;   // suppress re-triggering for a while after a measure
  SlopeEstimate slope_est_;        // live slope estimate (this tick's forces)
  SlopeEstimate measured_slope_;   // latched estimate from the last MEASURING window
  bool ik_ok_[cfg::NUM_FEET] = {true, true, true, true};
  float probe_[cfg::NUM_FEET] = {0, 0, 0, 0};      // late-contact ground-search reach (mm)
  float late_time_[cfg::NUM_FEET] = {0, 0, 0, 0};  // sustained late-contact time (probe debounce)

  RobotState state_ = RobotState::SLEEP;
  float phase_ = 0.0f;                       // global gait cycle phase [0,1)
  float local_phase_[cfg::NUM_FEET] = {0};   // per-leg phase, for ground-contact logic
  Vec3  foot_target_[cfg::NUM_FEET];         // this tick's foot targets (leg frame)
  float idle_timer_ = 0.0f;                  // time with no command (WALK → STAND)
  // Graceful start/stop state.
  float stride_scale_ = 1.0f;                // gait amplitude, ramps 1→0 while stopping
  float start_lift_ = 1.0f;                  // swing-lift ease-in at walk start (0→1)
  float walk_cadence_ = 0.0f;                // last walking cadence (cycles/s), reused to wind down
  float stop_timer_ = 0.0f;                  // time spent in STOPPING
  Command last_cmd_;                         // last walking command (direction for wind-down steps)
  float last_good_[cfg::NUM_SERVOS] = {0};   // last successful servo solution
  bool  have_last_good_ = false;
};

}  // namespace koda
