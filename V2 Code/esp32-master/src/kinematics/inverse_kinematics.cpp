#include "kinematics/inverse_kinematics.h"

#include <cmath>

#include "config.h"
#include "math/angle.h"

namespace koda {

bool inverse_kinematics(const Vec3& foot, JointAngles& out) {
  const float x = foot.x;
  const float y = foot.y;
  const float z = foot.z;

  const float a            = cfg::LEG_GEOMETRY_A_MM;     // 31.56 — see config.h note
  const float thigh        = cfg::THIGH_LENGTH_MM;
  const float calf         = cfg::CALF_LENGTH_MM;
  const float servo_dist   = cfg::SERVO_DISTANCE_MM;

  // ── Hip rotation (abduction) — projects the 3D target into the leg's sagittal plane.
  const float q = cfg::HIP_SEPARATION_MM / 2.0f - z;
  const float under_root = q * q + y * y - a * a;
  if (under_root < 0.0f) return false;                   // target inside the hip offset
  const float y_prime = std::sqrt(under_root);
  // V1 used single-arg atan(y/q) here (not atan2) — faithfully reproduced so the hip
  // angle matches what the servo-mapping was tuned against.
  const float theta_h = rad2deg(PI_F - std::atan(y / q) - std::atan(y_prime / a));

  // ── Sagittal 5-bar: foot at (x, y_prime), two knee servos either side of the hip.
  const Vec2 foot2{x, y_prime};
  const Vec2 servo1{-servo_dist / 2.0f, 0.0f};
  const Vec2 servo2{ servo_dist / 2.0f, 0.0f};

  Vec2 s1[2], s2[2];
  // Knee = intersection of the calf circle (centre foot, r=calf) with the thigh circle
  // (centre servo, r=thigh).
  if (circle_intersection(foot2, calf, servo1, thigh, s1) != 2) return false;
  if (circle_intersection(foot2, calf, servo2, thigh, s2) != 2) return false;

  // Pick the correct knee of the two candidates (V1's >180° elbow-side test).
  const Vec2 knee1 =
      (clockwise_angle_deg(servo1, foot2, s1[0]) < 180.0f) ? s1[1] : s1[0];
  const Vec2 knee2 =
      (clockwise_angle_deg(servo2, foot2, s2[0]) < 180.0f) ? s2[0] : s2[1];

  out.hip   = theta_h;
  out.left  = counterclockwise_angle_deg(servo2, knee1, servo1);
  out.right = clockwise_angle_deg(servo1, knee2, servo2);
  return true;
}

}  // namespace koda
