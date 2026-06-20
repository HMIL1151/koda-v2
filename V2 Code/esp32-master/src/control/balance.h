// Static balance / COG management on an incline.
//
// When the robot stands still on a slope two things go wrong: the body tilts with the
// ground, and the centre of gravity drifts toward the downhill feet (which then carry
// most of the load and risk toppling). This controller corrects both, every tick, while
// stationary:
//
//   1. Level the torso. Using the IMU tilt (if fitted) it counter-rotates the body so
//      the torso stays level with gravity rather than with the slope.
//
//   2. Re-centre the COG. The hall-effect foot forces reveal the load distribution —
//      front vs rear and left vs right. The controller shifts the body horizontally to
//      even those out, pulling the COG back over the middle of the support polygon.
//
// Both outputs are proportional, clamped (config.h BALANCE_MAX_*), and smoothed so the
// body eases into the correction instead of twitching with sensor noise. The result is a
// BodyPose the robot applies on top of its neutral stance.
#pragma once

#include "control/body_pose.h"
#include "sensors/imu.h"

namespace koda {

class Balance {
 public:
  // ── Static path (live force feedback) ──────────────────────────────────────────────
  // forces : per-foot ground reaction force (N), order FL, FR, RR, RL.
  // tilt   : torso tilt from the IMU; pass a level Tilt{} when no IMU is present and the
  //          controller falls back to foot-force re-centring only.
  // dt     : seconds since the last call (the COG re-centring integrates over time).
  // Used while STANDING, where the springs give a trustworthy continuous reading.
  BodyPose update(const float* forces, Tilt tilt, float dt);

  // ── Walking path (feed-forward) ────────────────────────────────────────────────────
  // The hall sensors are too slow to balance on live force while walking, so WALK biases
  // the COG from the last STATIC slope measurement instead. set_slope() stores it;
  // feedforward_pose() returns the COG shift that keeps it over the support centroid.
  void set_slope(float pitch_rad, float roll_rad) {
    slope_pitch_ = pitch_rad;
    slope_roll_ = roll_rad;
  }
  BodyPose feedforward_pose() const;

  void reset() { pose_ = BodyPose::identity(); }
  const BodyPose& pose() const { return pose_; }

 private:
  BodyPose pose_;             // smoothed live-feedback pose (static path)
  float slope_pitch_ = 0.0f;  // last measured slope (feed-forward path)
  float slope_roll_ = 0.0f;
};

}  // namespace koda
