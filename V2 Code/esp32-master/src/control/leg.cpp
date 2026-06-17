#include "control/leg.h"

#include "kinematics/inverse_kinematics.h"

namespace koda {

bool Leg::solve(const Vec3& foot, float servo_deg[cfg::NUM_SERVOS]) const {
  JointAngles k;
  if (!inverse_kinematics(foot, k)) return false;

  // Leg-convention mapping (V1 leg.py). The two knee drivers cross over and are mirrored
  // by `side`; the hip is mirrored by side·face. Physical trims are NOT applied here —
  // the slave's calibration table owns those.
  const float left_servo  = (k.right - cfg::SERVO_OFFSET_DEG) * static_cast<float>(side_);
  const float right_servo = (k.left  - cfg::SERVO_OFFSET_DEG) * static_cast<float>(-side_);
  const float hip_servo   = (side_ * face_ < 0) ? (-k.hip + 180.0f) : (k.hip - 180.0f);

  servo_deg[hip_ch_]    = hip_servo;
  servo_deg[knee_l_ch_] = left_servo;
  servo_deg[knee_r_ch_] = right_servo;
  return true;
}

}  // namespace koda
