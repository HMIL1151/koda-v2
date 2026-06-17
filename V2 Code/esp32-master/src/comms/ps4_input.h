// PS4 / DualShock input via Bluepad32, normalised into a Command.
//
// Bluepad32 owns the BLE HID host and hands us a controller object once one pairs. We
// translate sticks/buttons into the robot's Command and debounce the edge-triggered
// buttons here so the control code only sees clean one-tick pulses.
#pragma once

#include "control/command.h"

namespace koda {

class PS4Input {
 public:
  void begin();              // start Bluepad32, install connection callbacks
  Command poll();            // call once per tick; returns the latest command

 private:
  // Previous button levels, for edge detection.
  bool prev_stand_ = false;
  bool prev_sit_   = false;
  bool prev_gait_  = false;
  bool prev_cal_   = false;   // L1+R1 combo
  bool prev_circle_ = false;  // Circle (cancel)
};

}  // namespace koda
