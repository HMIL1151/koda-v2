// Normalised operator command — the single struct the rest of the control code consumes,
// independent of where it came from (PS4 today, something else tomorrow). PS4Input fills
// it; Robot reads it.
#pragma once

namespace koda {

struct Command {
  // Continuous axes, each normalised to [-1, 1] with deadzone already applied.
  float vx     = 0.0f;   // forward / back   (+ forward)
  float vy     = 0.0f;   // strafe           (+ right)
  float yaw    = 0.0f;   // turn rate        (+ clockwise from above)
  float height = 0.0f;   // body height trim (+ taller)

  // Edge-triggered buttons (true for the one tick they're pressed).
  bool stand_toggle = false;   // sleep ⇄ stand
  bool sit_toggle   = false;   // sit
  bool gait_cycle   = false;   // next gait (trot → crawl → gallop → …)

  // Hall-sensor calibration (modal — see HallCalibrator).
  bool cal_enter   = false;    // L1+R1: enter calibration mode
  bool cal_confirm = false;    // Cross: capture this step / advance
  bool cal_cancel  = false;    // Circle: abort calibration

  bool connected = false;      // is a controller actually paired?
};

}  // namespace koda
