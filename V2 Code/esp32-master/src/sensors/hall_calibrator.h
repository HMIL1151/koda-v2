// Guided hall-sensor calibration — the on-robot version of the bench sketch's
// "press ENTER" flow. A small state machine walks the operator through capturing the
// unloaded and fully-compressed signals, solves K for every sensor, and saves the result
// to flash. It advances on either a controller button or a Serial newline, so it works
// on the bench (USB) and in the field (PS4).
//
// While calibrating, main() holds the servos in RELAX so the legs move freely by hand.
#pragma once

#include "sensors/hall_sensor.h"

namespace koda {

class HallCalibrator {
 public:
  explicit HallCalibrator(HallSensors& hall) : hall_(hall) {}

  void start();                            // enter calibration, prompt for zero load
  void cancel();                           // abort, leave existing calibration intact
  bool active() const { return state_ != State::Idle; }

  // Per tick while active. `confirm`/`cancel` are edge-triggered controller buttons; a
  // Serial newline also confirms. Returns true on the tick calibration completes & saves.
  bool update(bool confirm, bool cancel);

 private:
  enum class State { Idle, WaitZero, WaitFull };

  bool serial_confirmed();                 // true if a newline arrived on Serial

  HallSensors& hall_;
  State state_ = State::Idle;
};

}  // namespace koda
