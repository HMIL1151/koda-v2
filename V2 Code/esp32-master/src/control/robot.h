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
#include "gait/gait.h"
#include "protocol/protocol.h"
#include "sensors/imu.h"

namespace koda {

enum class RobotState { SLEEP, STAND, WALK, SIT };

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

 private:
  // Map a body pose + height trim onto a leg's neutral foot target (leg frame).
  Vec3 neutral_foot(int leg, const BodyPose& pose, float height_trim) const;
  // Per-leg ground-plane step vector for the current command.
  Vec2 step_vector(int leg, const Command& cmd) const;
  // Solve all four legs into `out`, holding last-good angles on any IK miss.
  void solve_all(const Vec3 feet[cfg::NUM_FEET], float out[cfg::NUM_SERVOS]);
  void cycle_gait();

  Leg legs_[cfg::NUM_FEET];
  Gait gait_;
  Balance balance_;

  RobotState state_ = RobotState::SLEEP;
  float phase_ = 0.0f;                       // global gait cycle phase [0,1)
  float local_phase_[cfg::NUM_FEET] = {0};   // per-leg phase, for ground-contact logic
  float idle_timer_ = 0.0f;                  // time with no command (WALK → STAND)
  float last_good_[cfg::NUM_SERVOS] = {0};   // last successful servo solution
  bool  have_last_good_ = false;
};

}  // namespace koda
