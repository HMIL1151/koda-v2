#include "sensors/imu.h"

#include <cmath>

#include "config.h"

namespace koda {

bool Imu::begin() {
  if (!cfg::HAS_IMU) {
    present_ = false;
    return false;
  }
  // TODO: initialise your I2C IMU here (Wire.begin(); device wake/config).
  // Set present_ = true only once a known device id reads back correctly.
  present_ = false;
  return present_;
}

bool Imu::read_accel(float& ax, float& ay, float& az) {
  // TODO: read the accelerometer (in g) from the device. Left unimplemented until a
  // sensor is wired; reports "level" so balance falls back to foot-force only.
  ax = 0.0f;
  ay = 0.0f;
  az = 1.0f;
  return false;
}

void Imu::update() {
  if (!present_) {
    tilt_ = {};
    return;
  }
  float ax, ay, az;
  if (!read_accel(ax, ay, az)) return;        // keep last good tilt on a failed read

  // Gravity-vector tilt from the accelerometer (static / slow-moving assumption — fine
  // for the standing balance case this is used for).
  tilt_.pitch_rad = std::atan2(-ax, std::sqrt(ay * ay + az * az));
  tilt_.roll_rad  = std::atan2(ay, az);
}

}  // namespace koda
