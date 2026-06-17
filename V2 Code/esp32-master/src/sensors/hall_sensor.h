// Hall-effect foot-force sensing, inverse-cube magnetic model.
//
// Each calf is a series spring; a hall sensor watches a magnet on the moving spring end.
// The signal follows  signal = K / (distance + offset)^3  (ported from the bench
// calibration sketch). Per sensor we store a two-point calibration (s0 unloaded, s1 fully
// compressed) and solve K; from a live signal we recover spring distance, then force.
//
// There are two calf sensors per foot; a foot's force is the sum of its two sensors.
// Reads are round-robined across ticks (HALL_READS_PER_TICK) so one control tick never
// blocks on all eight slow ADC conversions.
#pragma once

#include "config.h"
#include "sensors/adc.h"

namespace koda {

// Per-sensor two-point calibration. s0/s1 are raw ADC counts; K is solved from them.
struct HallCal {
  float s0 = 0.0f;          // unloaded signal
  float s1 = 0.0f;          // fully-compressed signal
  float K = 1.0f;           // solved magnetic constant
  bool  calibrated = false;
};

class HallSensors {
 public:
  explicit HallSensors(AnalogSource& adc) : adc_(adc) {}

  void begin();             // bring up the ADC source
  void update();            // round-robin sample + filter; call once per control tick

  float force_n(int foot) const { return foot_force_n_[foot]; }   // foot 0..3, newtons
  const float* forces() const { return foot_force_n_; }

  // ── Calibration support (used by HallCalibrator + HallStore) ───────────────────────
  // Careful averaged read of one sensor's raw signal (blocking — calibration only).
  float read_filtered(int sensor) const;
  // Capture the current signal of every sensor into s0 (unloaded) or s1 (full load).
  void capture_zero();
  void capture_full();
  void solve();             // solve K for every sensor from its s0/s1
  bool all_calibrated() const;

  HallCal& cal(int sensor) { return cal_[sensor]; }
  const HallCal& cal(int sensor) const { return cal_[sensor]; }

 private:
  float signal_to_distance(float signal, const HallCal& c) const;
  float distance_to_force(float distance) const;
  void  update_sensor_force(int sensor);

  AnalogSource& adc_;
  HallCal cal_[cfg::NUM_HALL_SENSORS];
  float sensor_force_n_[cfg::NUM_HALL_SENSORS] = {0};   // EMA-filtered per sensor
  float foot_force_n_[cfg::NUM_FEET] = {0};             // summed per foot
  int   next_sensor_ = 0;                               // round-robin cursor
  bool  primed_ = false;
};

}  // namespace koda
