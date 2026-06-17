#include "control/balance.h"

#include "config.h"
#include "math/angle.h"
#include "math/vec.h"

namespace koda {

namespace {
constexpr float kSmoothing = 0.20f;     // EMA toward the target correction (ease-in)
constexpr float kEps = 1e-3f;

// Leg/force order: FL, FR, RR, RL.
constexpr int FL = 0, FR = 1, RR = 2, RL = 3;
}  // namespace

BodyPose Balance::update(const float* forces, Tilt tilt) {
  // ── Load distribution from the foot forces ────────────────────────────────────────
  const float front = forces[FL] + forces[FR];
  const float rear  = forces[RR] + forces[RL];
  const float left  = forces[FL] + forces[RL];
  const float right = forces[FR] + forces[RR];

  BodyPose target;

  // 2) Re-centre the COG: shift the body toward whichever side is under-loaded so the
  //    forces even out. Gains are mm-per-newton (config.h), output clamped.
  //    NOTE: if the body moves the *wrong* way on the real robot, flip these two signs.
  target.translation.x =
      clampf(cfg::BALANCE_KP_SHIFT * (rear - front),
             -cfg::BALANCE_MAX_SHIFT_MM, cfg::BALANCE_MAX_SHIFT_MM);
  target.translation.z =
      clampf(cfg::BALANCE_KP_SHIFT * (left - right),
             -cfg::BALANCE_MAX_SHIFT_MM, cfg::BALANCE_MAX_SHIFT_MM);
  target.translation.y = 0.0f;

  // 1) Level the torso: counter-rotate to drive the measured tilt back toward zero.
  //    With no IMU, tilt is zero and these stay zero (force re-centring still works).
  const float max_tilt = deg2rad(cfg::BALANCE_MAX_TILT_DEG);
  target.pitch_rad =
      clampf(-cfg::BALANCE_KP_PITCH * tilt.pitch_rad, -max_tilt, max_tilt);
  target.roll_rad =
      clampf(-cfg::BALANCE_KP_ROLL * tilt.roll_rad, -max_tilt, max_tilt);
  target.yaw_rad = 0.0f;

  // ── Ease the live pose toward the target so corrections are smooth, not twitchy ────
  pose_.translation.x = lerp(pose_.translation.x, target.translation.x, kSmoothing);
  pose_.translation.y = lerp(pose_.translation.y, target.translation.y, kSmoothing);
  pose_.translation.z = lerp(pose_.translation.z, target.translation.z, kSmoothing);
  pose_.pitch_rad     = lerp(pose_.pitch_rad,     target.pitch_rad,     kSmoothing);
  pose_.roll_rad      = lerp(pose_.roll_rad,      target.roll_rad,      kSmoothing);
  pose_.yaw_rad       = lerp(pose_.yaw_rad,       target.yaw_rad,       kSmoothing);

  (void)kEps;
  return pose_;
}

}  // namespace koda
