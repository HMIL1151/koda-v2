// Pure hall-effect force model — no hardware, no Arduino. Shared by the firmware
// (sensors/hall_sensor.cpp) and the off-hardware simulator (compiled to WASM), so both use
// the exact same signal→force maths. Inverse-cube magnetic law from the bench sketch:
//
//     signal = K / (distance + offset)^3
//
// Calibration is two points: s0 (unloaded) and s1 (fully compressed); K solves from them.
#pragma once

#include <cmath>

#include "config.h"
#include "math/vec.h"   // clampf

namespace koda {

// Per-sensor two-point calibration. s0/s1 are raw ADC counts; K solved from them.
struct HallCal {
  float s0 = 0.0f;          // unloaded signal
  float s1 = 0.0f;          // fully-compressed signal
  float K = 1.0f;           // solved magnetic constant
  bool  calibrated = false;
};

namespace hall_model {

// Recover spring distance (mm) from a live signal and a sensor's calibration.
inline float signal_to_distance(float signal, const HallCal& c) {
  if (!c.calibrated) return cfg::HALL_ZERO_LOAD_DIST_MM;

  const float s_lo = std::fmin(c.s0, c.s1);
  const float s_hi = std::fmax(c.s0, c.s1);
  signal = clampf(signal, s_lo, s_hi);

  const float d0 = cfg::HALL_ZERO_LOAD_DIST_MM + cfg::HALL_MAGNET_OFFSET_MM;
  float inv_d_cubed = (signal - c.s0) / c.K + 1.0f / (d0 * d0 * d0);
  if (inv_d_cubed <= 0.0f) inv_d_cubed = 1e-9f;     // numerical safety

  const float effective = std::cbrt(1.0f / inv_d_cubed);
  const float physical = effective - cfg::HALL_MAGNET_OFFSET_MM;
  return clampf(physical, cfg::HALL_FULL_LOAD_DIST_MM, cfg::HALL_ZERO_LOAD_DIST_MM);
}

// Spring distance (mm) → compressive force (N). Never negative.
inline float distance_to_force(float distance) {
  const float force = (cfg::HALL_ZERO_LOAD_DIST_MM - distance) * cfg::HALL_SPRING_N_PER_MM;
  return force < 0.0f ? 0.0f : force;
}

// Convenience: raw signal → force (N).
inline float force_from_signal(float signal, const HallCal& c) {
  return distance_to_force(signal_to_distance(signal, c));
}

// Solve K for a sensor from its captured s0/s1, and mark it calibrated.
inline void solve_k(HallCal& c) {
  const float d0 = cfg::HALL_ZERO_LOAD_DIST_MM + cfg::HALL_MAGNET_OFFSET_MM;
  const float d1 = cfg::HALL_FULL_LOAD_DIST_MM + cfg::HALL_MAGNET_OFFSET_MM;
  const float denom = 1.0f / (d1 * d1 * d1) - 1.0f / (d0 * d0 * d0);
  c.K = (denom != 0.0f) ? (c.s1 - c.s0) / denom : 1.0f;
  c.calibrated = true;
}

}  // namespace hall_model
}  // namespace koda
