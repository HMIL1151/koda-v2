#include "sensors/hall_sensor.h"

#include <Arduino.h>

#include <cmath>

#include "math/vec.h"

namespace koda {

void HallSensors::begin() {
  adc_.begin();
  // Calibration (s0/s1/K) is loaded from flash by HallStore before/after begin(); leave
  // it untouched here.
}

// ── Careful averaged read (calibration) ────────────────────────────────────────────
float HallSensors::read_filtered(int sensor) const {
  long total = 0;
  for (int i = 0; i < cfg::HALL_FILTER_SAMPLES; ++i) {
    total += adc_.read_raw(sensor);
    delayMicroseconds(cfg::HALL_SAMPLE_DELAY_US);
  }
  return static_cast<float>(total) / cfg::HALL_FILTER_SAMPLES;
}

// ── Model: signal → spring distance → force (ported from the bench sketch) ──────────
float HallSensors::signal_to_distance(float signal, const HallCal& c) const {
  if (!c.calibrated) return cfg::HALL_ZERO_LOAD_DIST_MM;

  // Clamp to the calibrated signal span.
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

float HallSensors::distance_to_force(float distance) const {
  const float compression = cfg::HALL_ZERO_LOAD_DIST_MM - distance;
  const float force = compression * cfg::HALL_SPRING_N_PER_MM;
  return force < 0.0f ? 0.0f : force;
}

// ── Per-tick update (round-robin so we never block on all 8 conversions) ────────────
void HallSensors::update_sensor_force(int sensor) {
  const float raw = static_cast<float>(adc_.read_raw(sensor));
  const float force = distance_to_force(signal_to_distance(raw, cal_[sensor]));
  sensor_force_n_[sensor] = primed_
      ? lerp(sensor_force_n_[sensor], force, cfg::HALL_EMA_ALPHA)
      : force;
}

void HallSensors::update() {
  for (int n = 0; n < cfg::HALL_READS_PER_TICK; ++n) {
    update_sensor_force(next_sensor_);
    next_sensor_ = (next_sensor_ + 1) % cfg::NUM_HALL_SENSORS;
  }
  primed_ = true;

  // Per-foot force = sum of that foot's two calf sensors.
  for (int f = 0; f < cfg::NUM_FEET; ++f) {
    float sum = 0.0f;
    for (int k = 0; k < cfg::SENSORS_PER_FOOT; ++k)
      sum += sensor_force_n_[f * cfg::SENSORS_PER_FOOT + k];
    foot_force_n_[f] = sum;
  }
}

// ── Calibration ─────────────────────────────────────────────────────────────────────
void HallSensors::capture_zero() {
  for (int s = 0; s < cfg::NUM_HALL_SENSORS; ++s) cal_[s].s0 = read_filtered(s);
}

void HallSensors::capture_full() {
  for (int s = 0; s < cfg::NUM_HALL_SENSORS; ++s) cal_[s].s1 = read_filtered(s);
}

void HallSensors::solve() {
  const float d0 = cfg::HALL_ZERO_LOAD_DIST_MM + cfg::HALL_MAGNET_OFFSET_MM;
  const float d1 = cfg::HALL_FULL_LOAD_DIST_MM + cfg::HALL_MAGNET_OFFSET_MM;
  const float inv0 = 1.0f / (d0 * d0 * d0);
  const float inv1 = 1.0f / (d1 * d1 * d1);
  for (int s = 0; s < cfg::NUM_HALL_SENSORS; ++s) {
    const float denom = inv1 - inv0;
    cal_[s].K = (denom != 0.0f) ? (cal_[s].s1 - cal_[s].s0) / denom : 1.0f;
    cal_[s].calibrated = true;
  }
}

bool HallSensors::all_calibrated() const {
  for (int s = 0; s < cfg::NUM_HALL_SENSORS; ++s)
    if (!cal_[s].calibrated) return false;
  return true;
}

}  // namespace koda
