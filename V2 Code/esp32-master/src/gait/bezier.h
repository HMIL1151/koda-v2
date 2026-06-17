// Bezier evaluation for swing-phase foot paths. V1 used the Bernstein form with
// binomial coefficients (bezier_curve.py); here we use De Casteljau, which needs no
// factorials, is numerically tame, and is cheap for the ≤6 control points we use.
#pragma once

#include "math/vec.h"

namespace koda {

// Evaluate an n-point Bezier curve at t ∈ [0,1] via De Casteljau. `n` ≤ kMaxCtrl.
inline Vec3 bezier_eval(const Vec3* ctrl, int n, float t) {
  constexpr int kMaxCtrl = 8;
  Vec3 tmp[kMaxCtrl];
  for (int i = 0; i < n; ++i) tmp[i] = ctrl[i];
  for (int level = n - 1; level > 0; --level)
    for (int i = 0; i < level; ++i)
      tmp[i] = tmp[i] * (1.0f - t) + tmp[i + 1] * t;
  return tmp[0];
}

}  // namespace koda
