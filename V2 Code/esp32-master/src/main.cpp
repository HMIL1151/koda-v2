// Koda V2 master — entry point and fixed-rate control loop.
//
// Each tick (cfg::CONTROL_HZ):
//   poll PS4 → read foot forces + IMU → update ground contact → step the robot →
//   stream 12 servo angles to the Servo2040 → read back its status.
// Hall-sensor calibration is modal: when active it takes over the loop, relaxes the legs,
// and walks the operator through capturing/saving calibration. Nothing here blocks for
// long; servo smoothing happens on the slave between target updates.
#include <Arduino.h>

#include "comms/ps4_input.h"
#include "comms/servo_link.h"
#include "config.h"
#include "control/ground_contact.h"
#include "control/robot.h"
#include "sensors/adc.h"
#include "sensors/hall_calibrator.h"
#include "sensors/hall_sensor.h"
#include "sensors/hall_store.h"
#include "sensors/imu.h"

using namespace koda;

namespace {
PS4Input       g_ps4;
Ads1115Source  g_adc;
HallSensors    g_hall(g_adc);
HallCalibrator g_cal(g_hall);
Imu            g_imu;
GroundContact  g_contact;
Robot          g_robot;
ServoLink      g_link;

uint32_t         g_last_tick_us = 0;
proto::SlaveMode g_last_mode_sent = proto::RELAX;
bool             g_mode_initialised = false;

void send_mode_if_changed(proto::SlaveMode mode) {
  if (!g_mode_initialised || mode != g_last_mode_sent) {
    g_link.send_mode(mode);
    g_last_mode_sent = mode;
    g_mode_initialised = true;
  }
}

// Bench convenience: typing 'c' on the Serial monitor starts calibration even with no
// controller paired (mirrors the original sketch's USB workflow).
bool serial_cal_request() {
  bool req = false;
  while (Serial.available()) {
    const char ch = Serial.read();
    if (ch == 'c' || ch == 'C') req = true;
  }
  return req;
}
}  // namespace

void setup() {
  Serial.begin(cfg::LINK_BAUD);
  Serial.println("Koda V2 master booting");

  g_ps4.begin();
  g_hall.begin();
  if (hall_store::load(g_hall) && g_hall.all_calibrated()) {
    Serial.println("Hall calibration loaded from flash");
  } else {
    Serial.println("No valid hall calibration — press L1+R1 (or send 'c') to calibrate");
  }
  g_imu.begin();
  g_link.begin();
  g_robot.begin();

  g_last_tick_us = micros();
}

void loop() {
  const uint32_t now = micros();
  if (static_cast<uint32_t>(now - g_last_tick_us) < cfg::CONTROL_PERIOD_US) return;
  const float dt = static_cast<uint32_t>(now - g_last_tick_us) * 1e-6f;
  g_last_tick_us = now;

  const Command cmd = g_ps4.poll();
  g_imu.update();

  // ── Calibration mode (modal, highest priority) ────────────────────────────────────
  if (!g_cal.active() && (cmd.cal_enter || serial_cal_request())) {
    g_cal.start();
  }
  if (g_cal.active()) {
    send_mode_if_changed(proto::RELAX);   // legs limp so they move by hand
    g_link.ping();                        // keep the slave watchdog fed
    g_cal.update(cmd.cal_confirm, cmd.cal_cancel);
    g_link.poll();
    return;
  }

  // ── Normal control ────────────────────────────────────────────────────────────────
  // Controller gone → fail safe.
  if (!cmd.connected) {
    send_mode_if_changed(proto::SAFE);
    g_link.ping();
    g_link.poll();
    return;
  }

  g_hall.update();
  g_contact.update(g_hall.forces(), g_robot.local_phase(), g_robot.swing_fraction());

  float servo_deg[cfg::NUM_SERVOS];
  g_robot.update(cmd, g_hall.forces(), g_imu.tilt(), g_contact, dt, servo_deg);

  const proto::SlaveMode mode = g_robot.desired_slave_mode();
  send_mode_if_changed(mode);
  if (mode == proto::ACTIVE) {
    g_link.send_targets(servo_deg);
  } else {
    g_link.ping();
  }

  g_link.poll();
}
