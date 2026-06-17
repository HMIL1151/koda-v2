// Inverse kinematics for one leg — a 5-bar planar linkage driven by two knee servos,
// plus a hip-rotation angle. Ported from V1 inverse_kinematics.py.
//
// Given a foot target (x, y, z) in the leg frame it returns three *kinematic* joint
// angles in degrees: the hip rotation and the two knee-driver servo angles. Turning
// those into physical, per-leg servo commands (sign flips, offsets, trim) is the job of
// control/leg — this module is pure geometry and is leg-agnostic.
#pragma once

#include "math/vec.h"

namespace koda {

struct JointAngles {
  float hip   = 0.0f;   // hip rotation (theta_h)
  float left  = 0.0f;   // "servo1" knee driver
  float right = 0.0f;   // "servo2" knee driver
};

// Solve the leg. Returns false if the target is unreachable (no linkage solution),
// in which case `out` is left untouched. V1 raised on this case; here we fail soft so
// the control loop can hold the last good pose instead of crashing.
bool inverse_kinematics(const Vec3& foot, JointAngles& out);

}  // namespace koda
