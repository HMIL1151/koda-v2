// Per-pose calf geometry. Faithful JS port of the sagittal 5-bar in
// esp32-master/src/kinematics/inverse_kinematics.cpp (+ math/angle.h), so the sim can read the
// live calf angle θ at each foot's commanded pose and fold the real linkage projection into the
// spring force (k_foot = kSpring·Σcos²θ). See ADR 0001 / SPRING_SIZING.md.
//
// Geometry constants come from the WASM module where exported; HIP_SEPARATION_MM and
// LEG_GEOMETRY_A_MM aren't (older bindings), so they fall back to the config.h values.

const DEG = 180 / Math.PI;
const wrap2pi = (a) => ((a %= 2 * Math.PI) < 0 ? a + 2 * Math.PI : a);
const cw = (p1, p2, v) =>
  wrap2pi(Math.atan2(p1.y - v.y, p1.x - v.x) - Math.atan2(p2.y - v.y, p2.x - v.x)) * DEG;

function circleIntersection(c1, r1, c2, r2) {
  const dx = c2.x - c1.x, dy = c2.y - c1.y, d = Math.hypot(dx, dy);
  if (d <= 0 || d >= r1 + r2) return null;
  const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d), h2 = r1 * r1 - a * a;
  if (h2 < 0) return null;
  const h = Math.sqrt(h2), xm = c1.x + (a / d) * dx, ym = c1.y + (a / d) * dy;
  return [
    { x: xm - (h / d) * dy, y: ym + (h / d) * dx },   // matches C++ out[0]
    { x: xm + (h / d) * dy, y: ym - (h / d) * dx },   // matches C++ out[1]
  ];
}

// The two calf angles (deg from vertical) at a leg-frame foot target [x, y, z]. Returns null if
// the target is unreachable (same conditions the IK rejects).
export function calfAnglesDeg(foot, M) {
  const HIP = M?.HIP_SEPARATION_MM ?? 85.0;       // config HIP_SEPARATION_MM
  const A = M?.LEG_GEOMETRY_A_MM ?? 31.56;        // config LEG_GEOMETRY_A_MM
  const SERVO = M?.SERVO_DISTANCE_MM ?? 46.0;
  const THIGH = M?.THIGH_LENGTH_MM ?? 30.0;
  const CALF = M?.CALF_LENGTH_MM ?? 120.0;

  const [x, y, z] = foot;
  const q = HIP / 2 - z;
  const under = q * q + y * y - A * A;
  if (under < 0) return null;
  const yprime = Math.sqrt(under);

  const foot2 = { x, y: yprime };
  const servo1 = { x: -SERVO / 2, y: 0 }, servo2 = { x: SERVO / 2, y: 0 };
  const s1 = circleIntersection(foot2, CALF, servo1, THIGH);
  const s2 = circleIntersection(foot2, CALF, servo2, THIGH);
  if (!s1 || !s2) return null;

  const knee1 = cw(servo1, foot2, s1[0]) < 180 ? s1[1] : s1[0];
  const knee2 = cw(servo2, foot2, s2[0]) < 180 ? s2[0] : s2[1];

  const angle = (knee) => Math.atan2(Math.abs(foot2.x - knee.x), foot2.y - knee.y) * DEG;
  return [angle(knee1), angle(knee2)];
}

// Σcos²θ over the two calves at a leg-frame foot target — the factor that turns the physical
// spring rate into the per-foot vertical rate (k_foot = kSpring·S). Falls back to `fallback`
// (the standing-pose value) if the target is unreachable.
export function springSumCos2(foot, M, fallback) {
  const angles = calfAnglesDeg(foot, M);
  if (!angles) return fallback;
  const c1 = Math.cos(angles[0] / DEG), c2 = Math.cos(angles[1] / DEG);
  return c1 * c1 + c2 * c2;
}
