#include "control/terrain_monitor.h"

#include <cmath>

namespace koda {

namespace {
constexpr int FL = 0, FR = 1, RR = 2, RL = 3;
}  // namespace

void TerrainMonitor::reset() {
  for (int i = 0; i < cfg::NUM_FEET; ++i) {
    e_[i] = 0.0f;
    prev_early_[i] = prev_late_[i] = false;
  }
  slope_time_ = 0.0f;
  slope_confirmed_ = false;
}

void TerrainMonitor::update(const GroundContact& contact, float dt) {
  for (int i = 0; i < cfg::NUM_FEET; ++i) {
    // Rising edge of an early/late event nudges the per-leg bias; it fades otherwise.
    const bool early = contact.early_contact(i);
    const bool late = contact.late_contact(i);
    if (early && !prev_early_[i]) e_[i] += cfg::TERRAIN_EVENT_GAIN * (1.0f - e_[i]);
    if (late && !prev_late_[i]) e_[i] += cfg::TERRAIN_EVENT_GAIN * (-1.0f - e_[i]);
    prev_early_[i] = early;
    prev_late_[i] = late;
    e_[i] *= (1.0f - cfg::TERRAIN_DECAY_PER_S * dt);
  }

  // Confirm a sustained slope classification (the trigger for the measure cycle).
  if (classify() == Terrain::SLOPE) {
    slope_time_ += dt;
    if (slope_time_ >= cfg::TERRAIN_CONFIRM_S) slope_confirmed_ = true;
  } else {
    slope_time_ = 0.0f;
    slope_confirmed_ = false;
  }
}

float TerrainMonitor::pitch_bias() const {
  return 0.5f * (e_[FL] + e_[FR]) - 0.5f * (e_[RR] + e_[RL]);   // front − rear
}

float TerrainMonitor::roll_bias() const {
  return 0.5f * (e_[FR] + e_[RR]) - 0.5f * (e_[FL] + e_[RL]);   // right − left
}

float TerrainMonitor::energy() const {
  float s = 0.0f;
  for (int i = 0; i < cfg::NUM_FEET; ++i) s += std::fabs(e_[i]);
  return s / cfg::NUM_FEET;
}

TerrainMonitor::Terrain TerrainMonitor::classify() const {
  const float en = energy();
  if (en < cfg::TERRAIN_FLAT_ENERGY) return Terrain::FLAT;

  const float bias = std::fmax(std::fabs(pitch_bias()), std::fabs(roll_bias()));
  // A coherent directional bias (large, and explaining most of the event energy) = slope;
  // otherwise the events are scattered = uneven ground (handled per-step, no measure cycle).
  if (bias > cfg::TERRAIN_SLOPE_BIAS && bias / en > cfg::TERRAIN_COHERENCE) {
    return Terrain::SLOPE;
  }
  return Terrain::UNEVEN;
}

}  // namespace koda
