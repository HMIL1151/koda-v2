// Terrain classification from the consistency of early/late foot-contact events.
//
// While walking, the hall sensors are only fast enough for contact *events*, not live
// force. But the pattern of those events is informative: a slope produces a CONSISTENT
// directional bias (e.g. the front feet keep contacting early on an uphill, the rear late;
// or one side biased), whereas uneven ground scatters the events with no spatial pattern.
//
// Per leg we keep a fading bias e ∈ [-1,1]: an EARLY contact (ground came up sooner than
// planned) pushes it toward +1 (higher ground here), a LATE contact toward -1 (lower).
// Front-vs-rear and right-vs-left differences of e then give the slope bias; whether that
// bias explains most of the event energy distinguishes SLOPE from UNEVEN.
#pragma once

#include "config.h"
#include "control/ground_contact.h"

namespace koda {

class TerrainMonitor {
 public:
  enum class Terrain { FLAT, UNEVEN, SLOPE };

  void reset();
  void update(const GroundContact& contact, float dt);   // call each WALK tick

  Terrain classify() const;
  // True once a slope has been classified continuously for TERRAIN_CONFIRM_S — the trigger
  // for the automatic stop-and-measure cycle.
  bool slope_confirmed() const { return slope_confirmed_; }

  float pitch_bias() const;   // + = front feet see higher ground (uphill ahead)
  float roll_bias() const;    // + = right feet see higher ground

 private:
  float energy() const;

  float e_[cfg::NUM_FEET] = {0, 0, 0, 0};            // per-leg fading contact-timing bias
  bool prev_early_[cfg::NUM_FEET] = {false, false, false, false};
  bool prev_late_[cfg::NUM_FEET] = {false, false, false, false};
  float slope_time_ = 0.0f;                          // how long SLOPE has been classified
  bool slope_confirmed_ = false;
};

}  // namespace koda
