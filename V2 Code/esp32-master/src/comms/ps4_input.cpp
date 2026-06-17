#include "comms/ps4_input.h"

#include <Bluepad32.h>

#include <cmath>

namespace koda {

namespace {

ControllerPtr g_ctl = nullptr;     // the single controller we care about

void onConnected(ControllerPtr ctl) {
  if (g_ctl == nullptr) {
    g_ctl = ctl;
    Console.println("PS4: controller connected");
  }
}

void onDisconnected(ControllerPtr ctl) {
  if (g_ctl == ctl) {
    g_ctl = nullptr;
    Console.println("PS4: controller disconnected");
  }
}

// Stick axes arrive as roughly -512..511; normalise and deadzone.
float axis(int32_t raw) {
  constexpr float kDeadzone = 0.12f;
  const float v = static_cast<float>(raw) / 512.0f;
  return (std::fabs(v) < kDeadzone) ? 0.0f : v;
}

}  // namespace

void PS4Input::begin() {
  BP32.setup(&onConnected, &onDisconnected);
  BP32.enableVirtualDevice(false);
}

Command PS4Input::poll() {
  BP32.update();

  Command cmd;
  if (g_ctl == nullptr || !g_ctl->isConnected()) {
    cmd.connected = false;
    prev_stand_ = prev_sit_ = prev_gait_ = false;
    return cmd;
  }
  cmd.connected = true;

  // Left stick: translation. Right stick: yaw + body height.
  // Forward is stick-up, which reads negative, so negate Y axes.
  cmd.vx     = axis(-g_ctl->axisY());
  cmd.vy     = axis(g_ctl->axisX());
  cmd.yaw    = axis(g_ctl->axisRX());
  cmd.height = axis(-g_ctl->axisRY());

  // Buttons (Bluepad32 uses Xbox naming): y=Triangle, a=Cross, b=Circle, l1/r1=bumpers.
  const bool stand  = g_ctl->y();
  const bool sit    = g_ctl->a();
  const bool circle = g_ctl->b();
  const bool l1     = g_ctl->l1();
  const bool r1     = g_ctl->r1();

  // L1+R1 together = enter calibration; suppress gait-cycle while L1 is held so the combo
  // doesn't also fire R1's normal action.
  const bool cal_combo = l1 && r1;
  const bool gait = r1 && !l1;

  cmd.stand_toggle = stand && !prev_stand_;
  cmd.sit_toggle   = sit && !prev_sit_;
  cmd.gait_cycle   = gait && !prev_gait_;
  cmd.cal_enter    = cal_combo && !prev_cal_;
  cmd.cal_confirm  = sit && !prev_sit_;       // Cross doubles as "confirm step" (modal)
  cmd.cal_cancel   = circle && !prev_circle_;  // Circle aborts

  prev_stand_  = stand;
  prev_sit_    = sit;
  prev_gait_   = gait;
  prev_cal_    = cal_combo;
  prev_circle_ = circle;
  return cmd;
}

}  // namespace koda
