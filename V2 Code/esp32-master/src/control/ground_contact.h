// Early ground-contact detection.
//
// While a foot is in swing it should be in the air. If its hall-effect force sensor
// reports load before the swing finishes, the foot has met the ground early — an
// incline, a step, or an obstacle. We flag that so the gait can end the swing early and
// plant the foot instead of jamming it down a pre-planned trajectory.
//
// Per-foot state machine with force hysteresis (avoids chatter near the threshold):
//   airborne ──force > CONTACT_FORCE_N──▶ in_contact
//   in_contact ──force < CONTACT_RELEASE_N──▶ airborne
// "Early" contact additionally requires the leg to be in swing and past a minimum swing
// fraction (a foot just lifting off still has residual load).
#pragma once

#include "config.h"

namespace koda {

class GroundContact {
 public:
  // forces        : per-foot ground reaction force (N), from HallSensors.
  // local_phase   : per-leg phase in [0,1) (0..swing_fraction is swing).
  // swing_fraction: from the active gait.
  void update(const float* forces, const float* local_phase, float swing_fraction);

  bool in_contact(int leg) const { return in_contact_[leg]; }
  // True for one detection: foot loaded while still mid-swing. The gait should cut the
  // swing short for this leg.
  bool early_contact(int leg) const { return early_contact_[leg]; }

 private:
  bool in_contact_[cfg::NUM_FEET]    = {false, false, false, false};
  bool early_contact_[cfg::NUM_FEET] = {false, false, false, false};
};

}  // namespace koda
