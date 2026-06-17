// Angle helpers and 2-circle intersection — the geometric primitives the IK and gait
// need. Ported from V1's inverse_kinematics.py / units.py circle maths.
#pragma once

#include <cmath>
#include "math/vec.h"

namespace koda {

constexpr float PI_F   = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI_F;

inline float deg2rad(float d) { return d * (PI_F / 180.0f); }
inline float rad2deg(float r) { return r * (180.0f / PI_F); }

// Wrap to [0, 2π).
inline float wrap_2pi(float a) {
  a = std::fmod(a, TWO_PI);
  return a < 0.0f ? a + TWO_PI : a;
}

// Clockwise angle (degrees, 0..360) from line (vertex→p1) to line (vertex→p2).
// Mirrors V1 clockwise_angle_between_two_lines().
inline float clockwise_angle_deg(const Vec2& p1, const Vec2& p2, const Vec2& vertex) {
  const float t1 = std::atan2(p1.y - vertex.y, p1.x - vertex.x);
  const float t2 = std::atan2(p2.y - vertex.y, p2.x - vertex.x);
  return rad2deg(wrap_2pi(t1 - t2));
}

// Counter-clockwise variant. Mirrors V1 counterclockwise_angle_between_two_lines().
inline float counterclockwise_angle_deg(const Vec2& p1, const Vec2& p2, const Vec2& vertex) {
  const float t1 = std::atan2(p1.y - vertex.y, p1.x - vertex.x);
  const float t2 = std::atan2(p2.y - vertex.y, p2.x - vertex.x);
  return rad2deg(wrap_2pi(t2 - t1));
}

// Intersection of two circles. Writes up to 2 points into out[], returns the count
// (0, 1, or 2). Ported from V1 intersection_between_circles().
inline int circle_intersection(const Vec2& c1, float r1,
                               const Vec2& c2, float r2,
                               Vec2 out[2]) {
  const float dx = c2.x - c1.x;
  const float dy = c2.y - c1.y;
  const float d  = std::hypot(dx, dy);

  if (d <= 0.0f || d >= r1 + r2) return 0;          // concentric or too far apart

  const float a = (r1 * r1 - r2 * r2 + d * d) / (2.0f * d);
  const float h2 = r1 * r1 - a * a;
  if (h2 < 0.0f) return 0;                           // one inside the other
  const float h = std::sqrt(h2);

  const float xm = c1.x + a / d * dx;
  const float ym = c1.y + a / d * dy;

  out[0] = {xm - h / d * dy, ym + h / d * dx};
  out[1] = {xm + h / d * dy, ym - h / d * dx};
  return 2;
}

}  // namespace koda
