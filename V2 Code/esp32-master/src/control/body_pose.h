// A rigid-body pose offset for the torso, relative to its neutral stance. Shared by the
// balance controller (which produces corrections) and the robot (which applies them to
// foot targets). This is the V2 form of what V1 passed around as
// (translation, orientation) tuples in orientation.py.
#pragma once

#include "math/vec.h"

namespace koda {

struct BodyPose {
  Vec3  translation;        // body shift, mm:  x = forward, y = up, z = right
  float pitch_rad = 0.0f;   // nose up positive  (rotation in the x–y plane)
  float roll_rad  = 0.0f;   // right-side down positive (rotation in the z–y plane)
  float yaw_rad   = 0.0f;   // nose right positive (rotation in the x–z plane)

  static BodyPose identity() { return BodyPose{}; }
};

}  // namespace koda
