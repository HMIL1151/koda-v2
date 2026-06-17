// One leg: foot target → three logical servo angles.
//
// Combines the IK (geometry, leg-agnostic) with the leg-convention mapping that depends
// on which corner of the robot this leg is and how it's mirrored. This is V1's
// Leg.kinematic_angles_to_servo_angles(), minus the physical per-servo trims — those
// now live in the slave's calibration table, so the master stays purely about geometry.
#pragma once

#include "config.h"
#include "math/vec.h"

namespace koda {

class Leg {
 public:
  // side : cfg::LEFT / cfg::RIGHT,  face : cfg::FRONT / cfg::REAR.
  // hip_ch / knee_l_ch / knee_r_ch : this leg's three channels in the 12-servo vector.
  Leg(int side, int face, int hip_ch, int knee_l_ch, int knee_r_ch)
      : side_(side), face_(face),
        hip_ch_(hip_ch), knee_l_ch_(knee_l_ch), knee_r_ch_(knee_r_ch) {}

  // Solve a foot target (leg frame, mm) into the three servo angles (degrees) and write
  // them into `servo_deg` at this leg's channels. Returns false if the target is
  // unreachable, leaving `servo_deg` untouched so the caller can hold the last pose.
  bool solve(const Vec3& foot, float servo_deg[cfg::NUM_SERVOS]) const;

  int hip_channel() const { return hip_ch_; }
  int knee_l_channel() const { return knee_l_ch_; }
  int knee_r_channel() const { return knee_r_ch_; }

 private:
  int side_;
  int face_;
  int hip_ch_;
  int knee_l_ch_;
  int knee_r_ch_;
};

}  // namespace koda
