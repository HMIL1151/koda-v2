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

// ── Per-tick update (round-robin so we never block on all 8 conversions) ────────────
void HallSensors::update_sensor_force(int sensor) {
  const float raw = static_cast<float>(adc_.read_raw(sensor));
  const float force = hall_model::force_from_signal(raw, cal_[sensor]);
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
  for (int s = 0; s < cfg::NUM_HALL_SENSORS; ++s) hall_model::solve_k(cal_[s]);
}

bool HallSensors::all_calibrated() const {
  for (int s = 0; s < cfg::NUM_HALL_SENSORS; ++s)
    if (!cal_[s].calibrated) return false;
  return true;
}

}  // namespace koda
