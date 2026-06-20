#include "gait/gait.h"

#include "config.h"
#include "gait/bezier.h"
#include "math/vec.h"

namespace koda {

void Gait::set_type(GaitType type) {
  type_ = type;
  // Phase offsets are the normalised forms of V1's get_start_indices(), expressed as
  // fractions of the full cycle. Leg order: FL, FR, RR, RL.
  switch (type) {
    case GaitType::TROT:    // diagonal pairs: FL+RR, then FR+RL
      timing_ = {0.5f, {0.0f, 0.5f, 0.0f, 0.5f}};
      break;
    case GaitType::CRAWL:   // one foot at a time: FL, RL, FR, RR
      timing_ = {0.25f, {0.0f, 0.5f, 0.75f, 0.25f}};
      break;
    case GaitType::GALLOP:
      timing_ = {0.5f, {0.0f, 0.125f, 0.625f, 0.5f}};
      break;
  }
}

Vec3 Gait::foot_target(int leg_index, float phase, const Vec2& step, float height,
                       const Vec3& zero) const {
  // Per-leg phase, wrapped into [0,1).
  float p = phase + timing_.offset[leg_index];
  p -= static_cast<int>(p);            // fractional part (p ≥ 0 always here)

  const Vec3 half{step.x * 0.5f, 0.0f, step.y * 0.5f};
  const Vec3 over = half * (2.0f * cfg::STEP_CURVE_DELTA);   // = delta * full step
  const Vec3 lift{0.0f, -height, 0.0f};                       // smaller y = foot raised

  const float sf = timing_.swing_fraction;
  if (p < sf) {
    // ── Swing: 6-point bezier arc, back → up → front (ported from V1 control points).
    const float t = (sf > 0.0f) ? p / sf : 0.0f;
    const Vec3 ctrl[6] = {
        zero - half,             // P0  on ground, rear
        zero - half - over,      // P1  overshoot rear
        zero - half + lift,      // P2  lift, rear
        zero + half + lift,      // P3  lift, front
        zero + half + over,      // P4  overshoot front
        zero + half,             // P5  on ground, front
    };
    return bezier_eval(ctrl, 6, t);
  }

  // ── Stance: straight line front → rear, foot planted, driving the body forward.
  const float u = (sf < 1.0f) ? (p - sf) / (1.0f - sf) : 0.0f;
  const Vec3 front = zero + half;
  const Vec3 rear  = zero - half;
  return front + (rear - front) * u;
}

}  // namespace koda
