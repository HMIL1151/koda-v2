// Estimate ground slope (body pitch/roll vs gravity) from the calf-spring forces while the
// robot stands on a plane. This is the 3D form of the TorsoAngle sim's
// `inferFromCompressions`: at static equilibrium gravity passes through the force-weighted
// centre of pressure, so the COG's horizontal offset over the COG height gives the tilt.
//
// IMPORTANT: the load only reveals the slope while the COG is NOT yet shifted to balance it
// (a balanced stance loads every foot equally and carries no slope information). So this is
// read from a fixed, symmetric reference stance, before the COG re-centring is applied.
#pragma once

#include "math/vec.h"

namespace koda {

struct SlopeEstimate {
  float pitch_rad = 0.0f;   // nose-up positive (fore/aft slope)
  float roll_rad  = 0.0f;   // right-down positive (lateral slope)
  float confidence = 0.0f;  // 0..1: enough load, CoP inside the support
};

// forces    : per-foot ground reaction force (N), order FL, FR, RR, RL.
// foot_xz   : per-foot body-frame position, Vec2{x = forward, y = right (lateral)}.
// cog_height: COG height above the support plane (mm).
SlopeEstimate estimate_slope(const float* forces, const Vec2* foot_xz, float cog_height);

}  // namespace koda
