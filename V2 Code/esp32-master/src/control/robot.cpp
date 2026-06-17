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
constexpr float IDLE_TO_STAND_S   = 0.40f;   // still this long while walking → stand

// Leg body-frame positions (x forward, z right), used as lever arms for posture and as
// turn centres. Half-spans from the geometry.
constexpr float HALF_X = cfg::LEG_X_SEPARATION_MM * 0.5f;
constexpr float HALF_Z = cfg::ZERO_Z;                         // foot lateral half-spread
const Vec2 kLegBodyPos[cfg::NUM_FEET] = {
    { HALF_X, -HALF_Z},   // FL
    { HALF_X,  HALF_Z},   // FR
    {-HALF_X,  HALF_Z},   // RR
    {-HALF_X, -HALF_Z},   // RL
};

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
  for (int i = 0; i < cfg::NUM_FEET; ++i) local_phase_[i] = 0.5f;  // stance = not airborne
  balance_.reset();
}

Vec3 Robot::neutral_foot(int leg, const BodyPose& pose, float height_trim) const {
  Vec3 foot{cfg::ZERO_X, cfg::ZERO_Y + height_trim, cfg::ZERO_Z};

  // Body translation: moving the body +x/+z pushes the foot the other way in the leg
  // frame; raising the body (+y) extends the leg (larger y).
  foot.x -= pose.translation.x;
  foot.z -= pose.translation.z;
  foot.y += pose.translation.y;

  // Pitch / roll as leg-length changes about the lever arm (linearised posture model —
  // accurate enough for the small static corrections the balance controller makes).
  const Vec2 p = kLegBodyPos[leg];
  foot.y += p.x * std::sin(pose.pitch_rad);   // nose-up: front legs extend
  foot.y += p.y * std::sin(pose.roll_rad);    // right-down: right legs extend
  return foot;
}

Vec2 Robot::step_vector(int leg, const Command& cmd) const {
  // Linear travel: forward uses the long walk stride, strafe the shorter manoeuvre one.
  Vec2 step{cmd.vx * cfg::WALK_STEP_DIST_MM, cmd.vy * cfg::MANOUVRE_STEP_DIST_MM};

  // Turn: each foot steps tangent to its circle about the body centre.
  // NOTE: lateral & turn signs depend on physical leg convention — V1 patched these with
  // per-side hip flips. Validate/flip on the robot if strafe or turn goes the wrong way.
  const Vec2 r = kLegBodyPos[leg];
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

  for (int i = 0; i < cfg::NUM_FEET; ++i) legs_[i].solve(feet[i], out);

  std::memcpy(last_good_, out, sizeof(last_good_));
  have_last_good_ = true;
}

void Robot::update(const Command& cmd, const float* forces, Tilt tilt,
                   const GroundContact& contact, float dt,
                   float out[cfg::NUM_SERVOS]) {
  (void)forces;  // forces reach balance via the controller below

  // ── Button-driven state transitions ───────────────────────────────────────────────
  if (cmd.stand_toggle) {
    state_ = (state_ == RobotState::SLEEP) ? RobotState::STAND : RobotState::SLEEP;
    balance_.reset();
  }
  if (cmd.sit_toggle && state_ != RobotState::SLEEP) state_ = RobotState::SIT;
  if (cmd.gait_cycle) cycle_gait();

  const float mag = cmd_magnitude(cmd);

  // STAND ⇄ WALK driven by stick activity.
  if (state_ == RobotState::STAND && mag > CMD_DEADZONE) {
    state_ = RobotState::WALK;
    idle_timer_ = 0.0f;
  } else if (state_ == RobotState::WALK) {
    idle_timer_ = (mag < CMD_DEADZONE) ? idle_timer_ + dt : 0.0f;
    if (idle_timer_ >= IDLE_TO_STAND_S) state_ = RobotState::STAND;
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
      const BodyPose pose = balance_.update(forces, tilt);
      for (int i = 0; i < cfg::NUM_FEET; ++i) {
        feet[i] = neutral_foot(i, pose, height_trim);
        local_phase_[i] = 0.5f;            // standing = all feet planted
      }
      break;
    }

    case RobotState::WALK: {
      phase_ += CADENCE_HZ * mag * dt;
      phase_ -= std::floor(phase_);          // wrap to [0,1)
      const float sf = gait_.timing().swing_fraction;

      for (int i = 0; i < cfg::NUM_FEET; ++i) {
        float lp = phase_ + gait_.timing().offset[i];
        lp -= std::floor(lp);
        local_phase_[i] = lp;

        const Vec2 step = step_vector(i, cmd);
        if (contact.early_contact(i)) {
          // Foot hit the ground early (incline/obstacle): stop the swing and plant it at
          // the start of stance instead of driving it further down.
          feet[i] = gait_.foot_target(i, sf - gait_.timing().offset[i], step,
                                      cfg::STEP_HEIGHT_MM);
        } else {
          feet[i] = gait_.foot_target(i, phase_, step, cfg::STEP_HEIGHT_MM);
        }
        feet[i].y += height_trim;
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

  solve_all(feet, out);
}

proto::SlaveMode Robot::desired_slave_mode() const {
  return (state_ == RobotState::SLEEP) ? proto::RELAX : proto::ACTIVE;
}

}  // namespace koda
