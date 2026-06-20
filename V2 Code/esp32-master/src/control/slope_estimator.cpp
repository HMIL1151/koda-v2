#include "control/slope_estimator.h"

#include <cmath>

#include "config.h"
#include "math/vec.h"

namespace koda {

SlopeEstimate estimate_slope(const float* forces, const Vec2* foot_xz, float cog_height) {
  SlopeEstimate out;

  // Force-weighted centre of pressure (body frame) and total load.
  float total = 0.0f, cop_x = 0.0f, cop_z = 0.0f;
  for (int i = 0; i < cfg::NUM_FEET; ++i) {
    total += forces[i];
    cop_x += forces[i] * foot_xz[i].x;   // x = forward
    cop_z += forces[i] * foot_xz[i].y;   // y = right (lateral)
  }
  if (total < 1e-3f) return out;          // no load → no estimate (confidence 0)
  cop_x /= total;
  cop_z /= total;

  // Gravity passes through the CoP; the CoP offset over the COG height is the tilt.
  out.pitch_rad = std::atan2(cop_x, cog_height);
  out.roll_rad  = std::atan2(cop_z, cog_height);

  // Confidence: needs real load on the feet, and the CoP should sit inside the support
  // footprint (a CoP near/over an edge means it's near tipping → unreliable).
  float load_conf = total / cfg::SLOPE_MIN_MEASURE_FORCE_N;
  if (load_conf > 1.0f) load_conf = 1.0f;

  // Support half-spans from the foot positions (max |x|, max |z|).
  float half_x = 1e-3f, half_z = 1e-3f;
  for (int i = 0; i < cfg::NUM_FEET; ++i) {
    half_x = std::fmax(half_x, std::fabs(foot_xz[i].x));
    half_z = std::fmax(half_z, std::fabs(foot_xz[i].y));
  }
  const float margin_x = 1.0f - std::fabs(cop_x) / half_x;   // 1 = centred, 0 = at edge
  const float margin_z = 1.0f - std::fabs(cop_z) / half_z;
  float margin = std::fmin(margin_x, margin_z);
  margin = margin < 0.0f ? 0.0f : (margin > 1.0f ? 1.0f : margin);

  out.confidence = load_conf * margin;
  return out;
}

}  // namespace koda
