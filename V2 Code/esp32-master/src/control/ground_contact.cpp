#include "control/ground_contact.h"

namespace koda {

void GroundContact::update(const float* forces, const float* local_phase,
                           float swing_fraction) {
  for (int leg = 0; leg < cfg::NUM_FEET; ++leg) {
    const float f = forces[leg];

    // Hysteresis contact latch.
    if (!in_contact_[leg] && f > cfg::CONTACT_FORCE_N) {
      in_contact_[leg] = true;
    } else if (in_contact_[leg] && f < cfg::CONTACT_RELEASE_N) {
      in_contact_[leg] = false;
    }

    // Are we in the swing portion, and far enough through it to trust the load?
    const float p = local_phase[leg];
    const bool in_swing = p < swing_fraction;
    const float swing_progress = (swing_fraction > 0.0f) ? (p / swing_fraction) : 1.0f;

    early_contact_[leg] = in_swing &&
                          swing_progress >= cfg::EARLY_CONTACT_MIN_PHASE &&
                          in_contact_[leg];
  }
}

}  // namespace koda
