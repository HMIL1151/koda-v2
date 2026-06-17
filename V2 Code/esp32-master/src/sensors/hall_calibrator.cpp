#include "sensors/hall_calibrator.h"

#include <Arduino.h>

#include "config.h"
#include "sensors/hall_store.h"

namespace koda {

void HallCalibrator::start() {
  state_ = State::WaitZero;
  Serial.println();
  Serial.println("================ HALL CALIBRATION ================");
  Serial.println("Legs are now RELAXED — move them freely by hand.");
  Serial.println("Step 1/2: unload ALL springs (lift the robot / take");
  Serial.println("weight off the feet), then press X (or send Enter).");
  Serial.println("Press Circle (or reset) to cancel.");
}

void HallCalibrator::cancel() {
  state_ = State::Idle;
  Serial.println("Hall calibration cancelled — previous calibration kept.");
}

bool HallCalibrator::serial_confirmed() {
  bool got = false;
  while (Serial.available()) {
    if (Serial.read() == '\n') got = true;   // any newline confirms
  }
  return got;
}

bool HallCalibrator::update(bool confirm, bool cancel_btn) {
  if (state_ == State::Idle) return false;
  if (cancel_btn) {
    cancel();
    return false;
  }

  const bool go = confirm || serial_confirmed();
  if (!go) return false;

  if (state_ == State::WaitZero) {
    hall_.capture_zero();
    Serial.println();
    Serial.print("Captured unloaded signals (s0):");
    for (int s = 0; s < cfg::NUM_HALL_SENSORS; ++s) {
      Serial.print(' ');
      Serial.print(hall_.cal(s).s0, 1);
    }
    Serial.println();
    Serial.println("Step 2/2: compress ALL feet FULLY (bottom the");
    Serial.println("springs), then press X (or send Enter).");
    state_ = State::WaitFull;
    return false;
  }

  // State::WaitFull → capture, solve, save, finish.
  hall_.capture_full();
  hall_.solve();

  Serial.println();
  Serial.print("Captured full-load signals (s1):");
  for (int s = 0; s < cfg::NUM_HALL_SENSORS; ++s) {
    Serial.print(' ');
    Serial.print(hall_.cal(s).s1, 1);
  }
  Serial.println();
  Serial.println("Solved K per sensor:");
  for (int s = 0; s < cfg::NUM_HALL_SENSORS; ++s) {
    Serial.print("  sensor ");
    Serial.print(s);
    Serial.print(": K=");
    Serial.println(hall_.cal(s).K, 6);
  }

  const bool saved = hall_store::save(hall_);
  Serial.println(saved ? "Saved to " : "FAILED to save ");
  Serial.println(cfg::HALL_CAL_PATH);
  Serial.println("==================================================");

  state_ = State::Idle;
  return true;
}

}  // namespace koda
