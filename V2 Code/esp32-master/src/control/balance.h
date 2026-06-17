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
  // forces : per-foot ground reaction force (N), order FL, FR, RR, RL.
  // tilt   : torso tilt from the IMU; pass a level Tilt{} when no IMU is present and the
  //          controller falls back to foot-force re-centring only.
  BodyPose update(const float* forces, Tilt tilt);

  void reset() { pose_ = BodyPose::identity(); }
  const BodyPose& pose() const { return pose_; }

 private:
  BodyPose pose_;   // smoothed, persists between ticks
};

}  // namespace koda
