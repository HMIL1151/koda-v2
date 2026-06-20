#include "control/balance.h"

#include "config.h"
#include "math/angle.h"
#include "math/vec.h"

namespace koda {

namespace {
constexpr float kTiltSmoothing = 0.20f;   // EMA on the tilt-levelling term

// Leg/force order: FL, FR, RR, RL.
constexpr int FL = 0, FR = 1, RR = 2, RL = 3;
}  // namespace

BodyPose Balance::update(const float* forces, Tilt tilt, float dt) {
  // ── Load distribution from the foot forces ────────────────────────────────────────
  const float front = forces[FL] + forces[FR];
  const float rear  = forces[RR] + forces[RL];
  const float left  = forces[FL] + forces[RL];
  const float right = forces[FR] + forces[RR];

  // 1) Re-centre the COG (integral): shift the body until the loads equalise, i.e. until
  //    the COG sits over the support centroid. Integrating means a slope is fully
  //    corrected — the shift grows until the imbalance it senses is zero.
  //    NOTE: if the body moves the *wrong* way on the real robot, flip these two signs.
  pose_.translation.x = clampf(pose_.translation.x + cfg::BALANCE_KI_SHIFT * (rear - front) * dt,
                               -cfg::BALANCE_MAX_SHIFT_MM, cfg::BALANCE_MAX_SHIFT_MM);
  pose_.translation.z = clampf(pose_.translation.z + cfg::BALANCE_KI_SHIFT * (left - right) * dt,
                               -cfg::BALANCE_MAX_SHIFT_MM, cfg::BALANCE_MAX_SHIFT_MM);
  pose_.translation.y = 0.0f;

  // 2) Level the torso: counter-rotate to drive the measured tilt toward zero. With no
  //    IMU, tilt is zero and these stay zero (the force re-centring above still works).
  const float max_tilt = deg2rad(cfg::BALANCE_MAX_TILT_DEG);
  const float tp = clampf(-cfg::BALANCE_KP_PITCH * tilt.pitch_rad, -max_tilt, max_tilt);
  const float tr = clampf(-cfg::BALANCE_KP_ROLL * tilt.roll_rad, -max_tilt, max_tilt);
  pose_.pitch_rad = lerp(pose_.pitch_rad, tp, kTiltSmoothing);
  pose_.roll_rad  = lerp(pose_.roll_rad, tr, kTiltSmoothing);
  pose_.yaw_rad   = 0.0f;

  return pose_;
}

BodyPose Balance::feedforward_pose() const {
  // On a slope the COG projects downhill by cog_height·tan(slope); shift the body the other
  // way (uphill) by the same amount to hold the COG over the support centroid. Pure
  // feed-forward from the measured slope — no live force needed.
  BodyPose p;
  p.translation.x = clampf(-cfg::COG_HEIGHT_MM * std::tan(slope_pitch_) * cfg::WALK_FEEDFORWARD_GAIN,
                           -cfg::BALANCE_MAX_SHIFT_MM, cfg::BALANCE_MAX_SHIFT_MM);
  p.translation.z = clampf(-cfg::COG_HEIGHT_MM * std::tan(slope_roll_) * cfg::WALK_FEEDFORWARD_GAIN,
                           -cfg::BALANCE_MAX_SHIFT_MM, cfg::BALANCE_MAX_SHIFT_MM);
  p.translation.y = 0.0f;
  return p;
}

}  // namespace koda
