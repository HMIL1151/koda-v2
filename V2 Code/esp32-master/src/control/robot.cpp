#include "control/robot.h"

#include <cmath>
#include <cstring>

#include "math/angle.h"
#include "math/vec.h"

namespace koda {

namespace {
constexpr float HEIGHT_RANGE_MM   = 30.0f;   // body-height trim authority (cmd.height ±1)
constexpr float CADENCE_HZ        = 1.5f;    // gait cycles/sec at full stick
constexpr float CMD_DEADZONE      = 0.08f;   // below this magnitude = "no command"
constexpr float START_PHASE       = 0.25f;   // begin walking mid-swing (feet at centre) for a half-step start
constexpr float START_LIFT_RAMP_S = 0.15f;   // ease the swing lift in at start (avoid a foot pop)
constexpr float STOP_RAMP_S       = 0.45f;   // time to ramp stride 1→0 when stopping
constexpr float MIN_STOP_CADENCE  = 1.0f;    // floor on wind-down cadence (cycles/s)

// Hip fore/aft half-span (a fixed robot dimension). The lateral half-span comes from the
// tunable stance width, so it lives in leg_body_pos() below.
constexpr float HALF_X = cfg::LEG_X_SEPARATION_MM * 0.5f;
constexpr float LEG_SIGN_X[cfg::NUM_FEET] = {+1, +1, -1, -1};   // FL FR RR RL: front/rear
constexpr float LEG_SIGN_Z[cfg::NUM_FEET] = {-1, +1, +1, -1};   // left/right

float cmd_magnitude(const Command& c) {
  return std::fmax(std::fabs(c.vx), std::fmax(std::fabs(c.vy), std::fabs(c.yaw)));
}
}  // namespace

Robot::Robot()
    : legs_{
          Leg(cfg::LEFT,  cfg::FRONT, cfg::FL_HIP, cfg::FL_KNEE_L, cfg::FL_KNEE_R),
          Leg(cfg::RIGHT, cfg::FRONT, cfg::FR_HIP, cfg::FR_KNEE_L, cfg::FR_KNEE_R),
          Leg(cfg::RIGHT, cfg::REAR,  cfg::RR_HIP, cfg::RR_KNEE_L, cfg::RR_KNEE_R),
          Leg(cfg::LEFT,  cfg::REAR,  cfg::RL_HIP, cfg::RL_KNEE_L, cfg::RL_KNEE_R),
      },
      gait_(GaitType::TROT) {}

void Robot::begin() {
  state_ = RobotState::SLEEP;
  phase_ = 0.0f;
  for (int i = 0; i < cfg::NUM_FEET; ++i) {
    local_phase_[i] = 0.5f;   // stance = not airborne
    probe_[i] = 0.0f;
    late_time_[i] = 0.0f;
  }
  balance_.reset();
  terrain_.reset();
  measure_pending_ = false;
  measure_timer_ = 0.0f;
  measure_cooldown_ = 0.0f;
}

Vec2 Robot::leg_body_pos(int leg) const {
  return {LEG_SIGN_X[leg] * HALF_X, LEG_SIGN_Z[leg] * params_.stance_z};
}

float Robot::slope_follow(int leg) const {
  // Drop the foot onto the measured ground plane: a foot at fore/aft x and lateral z sees
  // ground at x·tan(pitch) + z·tan(roll) below the body centre. Extending the leg by that
  // (scaled by the gain) makes uphill feet ride higher and downhill feet reach down, so the
  // swing lands on the slope instead of early/late.
  const Vec2 p = leg_body_pos(leg);
  return slope_follow_gain_ * (p.x * std::tan(measured_slope_.pitch_rad) +
                               p.y * std::tan(measured_slope_.roll_rad));
}

float Robot::ground_probe(int leg, float dt, const GroundContact& contact) {
  if (local_phase_[leg] < gait_.timing().swing_fraction) {
    probe_[leg] = 0.0f;                           // swing: reset, search fresh next stance
    late_time_[leg] = 0.0f;
  } else if (contact.late_contact(leg)) {
    // In stance with no ground under the foot. Debounce so we react to a real drop, not a
    // momentary unload during a gait transition, then reach down to find the ground.
    late_time_[leg] += dt;
    if (late_time_[leg] >= cfg::LATE_PROBE_DELAY_S) {
      probe_[leg] = std::fmin(probe_[leg] + cfg::GROUND_PROBE_SPEED_MM_S * dt,
                              cfg::GROUND_PROBE_MAX_MM);
    }
  } else {
    late_time_[leg] = 0.0f;                        // loaded (or settling) → hold the reach found
  }
  return probe_[leg];
}

Vec3 Robot::apply_body_pose(Vec3 foot, int leg, const BodyPose& pose) const {
  // Body translation: moving the body +x/+z pushes the foot the other way in the leg frame;
  // raising the body (+y) extends the leg (larger y). The lateral (z) shift carries the
  // per-leg sign because the left/right legs are mirrored — a uniform leg-frame z change
  // would pinch the feet rather than shift the body sideways.
  foot.x -= pose.translation.x;
  foot.z -= pose.translation.z * LEG_SIGN_Z[leg];
  foot.y += pose.translation.y;

  // Pitch / roll as leg-length changes about the lever arm (linearised posture model —
  // accurate enough for the small corrections the balance controller makes).
  const Vec2 p = leg_body_pos(leg);
  foot.y += p.x * std::sin(pose.pitch_rad);   // nose-up: front legs extend
  foot.y += p.y * std::sin(pose.roll_rad);    // right-down: right legs extend
  return foot;
}

Vec3 Robot::neutral_foot(int leg, const BodyPose& pose, float height_trim) const {
  return apply_body_pose({params_.stance_x, params_.stance_y + height_trim, params_.stance_z},
                         leg, pose);
}

Vec3 Robot::gait_foot(int leg, const Command& cmd, float stride_scale, float height_scale,
                      const GroundContact& contact) const {
  const float sf = gait_.timing().swing_fraction;
  Vec2 step = step_vector(leg, cmd);
  step.x *= stride_scale;
  step.y *= stride_scale;
  const float height = params_.step_height * height_scale;
  const Vec3 zero = params_.stance();
  if (contact.early_contact(leg)) {
    // Foot hit the ground early (incline/obstacle): stop the swing and plant it at the
    // start of stance instead of driving it further down.
    return gait_.foot_target(leg, sf - gait_.timing().offset[leg], step, height, zero);
  }
  return gait_.foot_target(leg, phase_, step, height, zero);
}

Vec2 Robot::step_vector(int leg, const Command& cmd) const {
  // Linear travel: forward uses the tunable walk stride, strafe the shorter manoeuvre one.
  Vec2 step{cmd.vx * params_.step_len, cmd.vy * cfg::MANOUVRE_STEP_DIST_MM};

  // Turn: each foot steps tangent to its circle about the body centre.
  // NOTE: lateral & turn signs depend on physical leg convention — V1 patched these with
  // per-side hip flips. Validate/flip on the robot if strafe or turn goes the wrong way.
  const Vec2 r = leg_body_pos(leg);
  const float rn = r.length();
  if (rn > 1e-3f) {
    const Vec2 tangent{-r.y / rn, r.x / rn};
    step.x += tangent.x * cmd.yaw * cfg::MANOUVRE_STEP_DIST_MM;
    step.y += tangent.y * cmd.yaw * cfg::MANOUVRE_STEP_DIST_MM;
  }
  return step;
}

void Robot::cycle_gait() {
  switch (gait_.type()) {
    case GaitType::TROT:   gait_.set_type(GaitType::CRAWL);  break;
    case GaitType::CRAWL:  gait_.set_type(GaitType::GALLOP); break;
    case GaitType::GALLOP: gait_.set_type(GaitType::TROT);   break;
  }
}

void Robot::solve_all(const Vec3 feet[cfg::NUM_FEET], float out[cfg::NUM_SERVOS]) {
  // Start from the last good solution so an IK miss on one leg holds that leg's pose
  // rather than producing garbage.
  if (have_last_good_)
    std::memcpy(out, last_good_, sizeof(last_good_));
  else
    for (int i = 0; i < cfg::NUM_SERVOS; ++i) out[i] = 0.0f;

  for (int i = 0; i < cfg::NUM_FEET; ++i) ik_ok_[i] = legs_[i].solve(feet[i], out);

  std::memcpy(last_good_, out, sizeof(last_good_));
  have_last_good_ = true;
}

void Robot::update(const Command& cmd, const float* forces, Tilt tilt,
                   const GroundContact& contact, float dt,
                   float out[cfg::NUM_SERVOS]) {
  // Live slope estimate from the calf springs (only trustworthy from a static, unbalanced
  // reference stance — the MEASURING state latches it; elsewhere it's informational).
  Vec2 foot_xz[cfg::NUM_FEET];
  for (int i = 0; i < cfg::NUM_FEET; ++i) foot_xz[i] = leg_body_pos(i);
  slope_est_ = estimate_slope(forces, foot_xz, cfg::COG_HEIGHT_MM);

  // ── Button-driven state transitions ───────────────────────────────────────────────
  if (cmd.stand_toggle) {
    state_ = (state_ == RobotState::SLEEP) ? RobotState::STAND : RobotState::SLEEP;
    balance_.reset();
  }
  if (cmd.sit_toggle && state_ != RobotState::SLEEP) state_ = RobotState::SIT;
  if (cmd.gait_cycle) cycle_gait();

  const float mag = cmd_magnitude(cmd);
  if (measure_cooldown_ > 0.0f) measure_cooldown_ = std::fmax(0.0f, measure_cooldown_ - dt);

  if (measure_pending_) {
    // Auto stop-measure cycle in progress — drive it regardless of the stick. STOPPING
    // winds down to STAND on its own; STAND then hands off to the static MEASURING read.
    if (state_ == RobotState::STAND) {
      state_ = RobotState::MEASURING;
      measure_timer_ = 0.0f;
      balance_.reset();                       // freeze the COG shift so the load shows the slope
    }
  } else if (state_ == RobotState::STAND && mag > CMD_DEADZONE) {
    // Half-step start: begin the gait mid-swing (feet are centred under the hips there), so
    // the first move is a half-step FORWARD from the standstill pose — no leg slides
    // backward through stance against the floor. Full stride from the off (the half comes
    // from starting mid-cycle); only the swing lift is eased in so the feet don't pop.
    state_ = RobotState::WALK;
    phase_ = START_PHASE;
    start_lift_ = 0.0f;
    stride_scale_ = 1.0f;
  } else if (state_ == RobotState::WALK && mag < CMD_DEADZONE) {
    state_ = RobotState::STOPPING;
    stop_timer_ = 0.0f;                      // keep stride_scale_ at 1 and ramp from here
  } else if (state_ == RobotState::STOPPING && mag > CMD_DEADZONE) {
    state_ = RobotState::WALK;               // resume from wherever the wind-down got to
    start_lift_ = 1.0f;
    stride_scale_ = 1.0f;
  }

  Vec3 feet[cfg::NUM_FEET];
  const float height_trim = cmd.height * HEIGHT_RANGE_MM;

  switch (state_) {
    case RobotState::SLEEP: {
      // Relaxed crouch target (slave will have torque off, but keep a sane pose ready).
      for (int i = 0; i < cfg::NUM_FEET; ++i) {
        feet[i] = neutral_foot(i, BodyPose::identity(), -HEIGHT_RANGE_MM);
        local_phase_[i] = 0.5f;
      }
      break;
    }

    case RobotState::STAND: {
      // Static balance: shift COG + level torso from foot forces (+ IMU if present).
      const BodyPose pose = balance_.update(forces, tilt, dt);
      for (int i = 0; i < cfg::NUM_FEET; ++i) {
        feet[i] = neutral_foot(i, pose, height_trim);
        local_phase_[i] = 0.75f;           // standing = firmly in stance (so late contact can fire)
      }
      break;
    }

    case RobotState::WALK: {
      last_cmd_ = cmd;
      start_lift_ = std::fmin(1.0f, start_lift_ + dt / START_LIFT_RAMP_S);  // ease swing lift in
      walk_cadence_ = CADENCE_HZ * mag;
      phase_ += walk_cadence_ * dt;
      phase_ -= std::floor(phase_);          // wrap to [0,1)

      // Walking balance is FEED-FORWARD from the last measured slope (the hall sensors are
      // too slow to balance on live force while walking — they're used only for early/late
      // contact here). The slope is (re)measured when static; see the MEASURING state.
      const BodyPose pose = balance_.feedforward_pose();
      for (int i = 0; i < cfg::NUM_FEET; ++i) {
        float lp = phase_ + gait_.timing().offset[i];
        local_phase_[i] = lp - std::floor(lp);
        Vec3 f = gait_foot(i, cmd, 1.0f, start_lift_, contact);
        f.y += height_trim + slope_follow(i);     // pre-place the foot on the measured plane
        feet[i] = apply_body_pose(f, i, pose);
      }

      // Watch the early/late-contact pattern. A confirmed slope triggers the automatic
      // stop → measure → adjust → resume cycle (uneven ground is left to per-step handling).
      terrain_.update(contact, dt);
      if (auto_slope_ && measure_cooldown_ <= 0.0f && !measure_pending_ &&
          terrain_.slope_confirmed()) {
        measure_pending_ = true;
        state_ = RobotState::STOPPING;        // begin the graceful stop (takes effect next tick)
        stop_timer_ = 0.0f;
      }
      break;
    }

    case RobotState::MEASURING: {
      // Static slope read: hold the symmetric reference stance with NO balance shift, so the
      // load distribution reveals the slope, settle, then latch it and resume.
      measure_timer_ += dt;
      for (int i = 0; i < cfg::NUM_FEET; ++i) {
        feet[i] = neutral_foot(i, BodyPose::identity(), height_trim);
        local_phase_[i] = 0.75f;
      }
      if (measure_timer_ >= cfg::MEASURE_SETTLE_S) {
        set_measured_slope(slope_est_.pitch_rad, slope_est_.roll_rad);  // feed-forward bias
        measure_pending_ = false;
        measure_cooldown_ = cfg::MEASURE_COOLDOWN_S;
        terrain_.reset();
        if (mag > CMD_DEADZONE) {             // still commanded → resume walking (half-step start)
          state_ = RobotState::WALK;
          phase_ = START_PHASE;
          start_lift_ = 0.0f;
          stride_scale_ = 1.0f;
        } else {
          state_ = RobotState::STAND;
        }
      }
      break;
    }

    case RobotState::STOPPING: {
      // Graceful wind-down: keep the gait running while the stride ramps to zero, so the
      // feet ease into the neutral stance (finish the current step, then a half-step to
      // even up) instead of snapping there.
      stop_timer_ += dt;
      stride_scale_ = std::fmax(0.0f, stride_scale_ - dt / STOP_RAMP_S);
      const float cadence = std::fmax(walk_cadence_, MIN_STOP_CADENCE);
      phase_ += cadence * dt;
      phase_ -= std::floor(phase_);

      const BodyPose pose = balance_.feedforward_pose();   // feed-forward while winding down
      for (int i = 0; i < cfg::NUM_FEET; ++i) {
        float lp = phase_ + gait_.timing().offset[i];
        local_phase_[i] = lp - std::floor(lp);
        // Stop: scale both stride and lift by stride_scale_, so feet ease down to a planted
        // neutral stance (no bobbing in place) before the hand-off to STAND.
        Vec3 f = gait_foot(i, last_cmd_, stride_scale_, stride_scale_, contact);
        f.y += height_trim + slope_follow(i);
        feet[i] = apply_body_pose(f, i, pose);
      }

      // Done once the stride is fully wound down and at least one wind-down cycle has run
      // (so every foot has reached the neutral stance). Then it's a seamless hand-off to
      // STAND, which holds the same neutral pose.
      if (stride_scale_ <= 0.0f && stop_timer_ >= 1.0f / cadence) {
        state_ = RobotState::STAND;
      }
      break;
    }

    case RobotState::SIT: {
      // Simple sit: rear legs tuck/lower, fronts stay up. Placeholder pose — tune to taste.
      for (int i = 0; i < cfg::NUM_FEET; ++i) {
        feet[i] = neutral_foot(i, BodyPose::identity(), 0.0f);
        local_phase_[i] = 0.5f;
        const bool rear = (i == 2 || i == 3);   // RR, RL
        if (rear) {
          feet[i].y -= 40.0f;
          feet[i].x += 30.0f;
        }
      }
      break;
    }
  }

  // Late-contact ground probing: while a stance foot feels no ground, reach it down to find
  // it (rather than leaving the body unsupported). Active only when the robot is bearing
  // weight; reset otherwise. If the drop is beyond the leg's reach, IK flags that leg.
  const bool weight_bearing = state_ == RobotState::STAND || state_ == RobotState::WALK ||
                              state_ == RobotState::STOPPING || state_ == RobotState::MEASURING;
  for (int i = 0; i < cfg::NUM_FEET; ++i) {
    if (weight_bearing) {
      feet[i].y += ground_probe(i, dt, contact);
    } else {
      probe_[i] = 0.0f;
    }
  }

  for (int i = 0; i < cfg::NUM_FEET; ++i) foot_target_[i] = feet[i];
  solve_all(feet, out);
}

proto::SlaveMode Robot::desired_slave_mode() const {
  return (state_ == RobotState::SLEEP) ? proto::RELAX : proto::ACTIVE;
}

}  // namespace koda
