// Optional IMU → torso pitch/roll, for incline sensing in the balance controller.
//
// The robot can balance on a slope from foot forces alone, but an IMU makes incline
// estimation direct and drift-free. This is a thin interface: drop in a concrete driver
// (e.g. MPU6050/BNO055 over I2C) inside read_raw(). When cfg::HAS_IMU is false the whole
// thing reports level and costs nothing.
#pragma once

namespace koda {

struct Tilt {
  float pitch_rad = 0.0f;   // nose up positive
  float roll_rad  = 0.0f;   // right-side down positive
};

class Imu {
 public:
  bool begin();             // returns false if not present / disabled
  void update();            // call once per control tick
  Tilt tilt() const { return tilt_; }
  bool present() const { return present_; }

 private:
  // Fill ax/ay/az (g) from the actual sensor. Returns false if the read failed.
  bool read_accel(float& ax, float& ay, float& az);

  Tilt tilt_;
  bool present_ = false;
};

}  // namespace koda
