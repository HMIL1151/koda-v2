// 5-bar leg geometry for drawing. Given the foot position in the leg's sagittal plane,
// solve the two knee positions so the renderer can draw the real linkage (two knee servos
// → two thighs → two knees → two calves → shared foot) instead of a single line.
//
// This is forward *drawing* geometry (circle intersections), not control — the controller
// already decided the foot; we just reconstruct the mechanism that reaches it. Hip-local
// coords: x = fore/aft, y = downward (positive).

// Intersections of two circles. Returns [] (none) or [p, p].
function circleIntersect(c0, r0, c1, r1) {
  const dx = c1.x - c0.x, dy = c1.y - c0.y;
  const d = Math.hypot(dx, dy);
  if (d === 0 || d > r0 + r1 || d < Math.abs(r0 - r1)) return [];
  const a = (r0 * r0 - r1 * r1 + d * d) / (2 * d);
  const h2 = r0 * r0 - a * a;
  if (h2 < 0) return [];
  const h = Math.sqrt(h2);
  const xm = c0.x + (a / d) * dx, ym = c0.y + (a / d) * dy;
  return [
    { x: xm - (h / d) * dy, y: ym + (h / d) * dx },
    { x: xm + (h / d) * dy, y: ym - (h / d) * dx },
  ];
}

// Returns { servoF, servoR, kneeF, kneeR, foot } in hip-local sagittal coords, or null if
// the foot is unreachable. `foreaft`,`vertical` are the foot offset from the hip.
export function legLinkage(foreaft, vertical, M) {
  const servoF = { x: -M.SERVO_DISTANCE_MM / 2, y: 0 };  // front knee servo
  const servoR = { x: M.SERVO_DISTANCE_MM / 2, y: 0 };   // rear knee servo
  const foot = { x: foreaft, y: vertical };
  const thigh = M.THIGH_LENGTH_MM, calf = M.CALF_LENGTH_MM;

  const iF = circleIntersect(servoF, thigh, foot, calf);
  const iR = circleIntersect(servoR, thigh, foot, calf);
  if (!iF.length || !iR.length) return null;

  // Splay the knees outward (front knee to the front side, rear knee to the rear) so the
  // drawn linkage takes the natural 5-bar shape.
  const kneeF = iF[0].x < iF[1].x ? iF[0] : iF[1];
  const kneeR = iR[0].x > iR[1].x ? iR[0] : iR[1];
  return { servoF, servoR, kneeF, kneeR, foot };
}
