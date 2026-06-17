// Persist hall-sensor calibration to a JSON file on the ESP32's LittleFS flash, so the
// robot calibrates once and reloads on every boot. File path + version: config.h.
#pragma once

#include "sensors/hall_sensor.h"

namespace koda {
namespace hall_store {

// Load HALL_CAL_PATH and apply it to `hall`. Returns false if the file is missing,
// unparseable, the wrong version, or the wrong sensor count (→ calibration needed).
bool load(HallSensors& hall);

// Write the current calibration of every sensor to HALL_CAL_PATH. Returns false on a
// filesystem error.
bool save(const HallSensors& hall);

}  // namespace hall_store
}  // namespace koda
