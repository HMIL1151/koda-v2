// Small vector / math helpers for the simulator. DOM-free so it imports under node too.

export const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
export const lerp = (a, b, t) => a + (b - a) * t;

// 3D body→world rotation: roll about x, then pitch about z, then yaw about y.
// Sign conventions chosen so +pitch raises forward (+x) feet and the settle loop in
// world.js forms negative feedback (verified by the node sim test).
export function rotate3({ x, y, z }, pitch, roll, yaw) {
  // roll about x: (y,z)
  let y1 = y * Math.cos(roll) - z * Math.sin(roll);
  let z1 = y * Math.sin(roll) + z * Math.cos(roll);
  let x1 = x;
  // pitch about z: (x,y)
  let x2 = x1 * Math.cos(pitch) - y1 * Math.sin(pitch);
  let y2 = x1 * Math.sin(pitch) + y1 * Math.cos(pitch);
  let z2 = z1;
  // yaw about y: (x,z)
  let x3 = x2 * Math.cos(yaw) + z2 * Math.sin(yaw);
  let z3 = -x2 * Math.sin(yaw) + z2 * Math.cos(yaw);
  return { x: x3, y: y2, z: z3 };
}

export const add3 = (a, b) => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z });

// Shortest signed delta between two cycle phases in [0,1).
export function phaseDelta(now, prev) {
  let d = now - prev;
  if (d < -0.5) d += 1;
  if (d > 0.5) d -= 1;
  return d;
}
