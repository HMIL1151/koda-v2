// Gait generator. Turns a normalised cycle phase into a foot target in the leg frame.
//
// This is a cleaner reformulation of V1's gait.py + bezier_curve.py. V1 precomputed a
// whole array of foot positions whose *length* encoded the speed, then indexed into it
// with per-leg start offsets. That tangled speed, timing and geometry together.
//
// V2 instead parameterises the trajectory by a continuous phase ∈ [0,1):
//     • phase ∈ [0, swing_fraction)  → swing  (bezier arc, foot in the air)
//     • phase ∈ [swing_fraction, 1)  → stance (foot on ground, slides body-rearward)
// Speed becomes simply how fast `phase` advances each tick (owned by the caller), and
// each leg's phase is the global phase plus a fixed per-gait offset.
#pragma once

#include "math/vec.h"

namespace koda {

enum class GaitType { CRAWL, TROT, GALLOP };

// Per-gait timing: how much of the cycle each leg spends in swing, and the phase offset
// of each of the four legs (order: FL, FR, RR, RL — matching Robot's leg array).
struct GaitTiming {
  float swing_fraction;     // fraction of the cycle a leg is airborne
  float offset[4];          // phase offset per leg, in [0,1)
};

class Gait {
 public:
  explicit Gait(GaitType type = GaitType::TROT) { set_type(type); }

  void set_type(GaitType type);
  GaitType type() const { return type_; }
  const GaitTiming& timing() const { return timing_; }

  // Foot target for one leg at the given global cycle phase.
  //   leg_index : 0..3 (FL, FR, RR, RL)
  //   phase     : global cycle phase in [0,1)
  //   step      : ground-plane step vector for this leg this cycle, in the leg frame
  //               (x = forward/back, z = sideways). Its length is the stride length;
  //               its direction sets travel direction. Built by Robot from the command.
  //   height    : peak swing height (mm)
  // Returns the foot position in the leg frame (x, y, z), y measured like V1 (ZERO_Y
  // nominal, smaller y = foot lifted).
  Vec3 foot_target(int leg_index, float phase, const Vec2& step, float height) const;

 private:
  GaitType   type_;
  GaitTiming timing_;
};

}  // namespace koda
