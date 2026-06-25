// Tests for the live calf-angle port. Pure JS (no WASM). Run: node sim/test/calf_geometry.test.mjs
// Cross-checks against the faithful IK: at the ZERO standing target the two calves are symmetric
// at 24° (matching the C++ inverse_kinematics.cpp the firmware runs).

import { calfAnglesDeg, springSumCos2 } from '../web/calf_geometry.js';

let failures = 0;
const ok = (c, m) => (c ? console.log('  ok:', m) : (console.error('  FAIL:', m), failures++));

const ZERO = [0, 125, 85 / 2 + 31];   // ZERO_X, ZERO_Y, ZERO_Z (= 73.5)

console.log('Calf geometry:');
{
  const a = calfAnglesDeg(ZERO, null);
  ok(a !== null, 'standing target is reachable');
  ok(Math.abs(a[0] - 24.0) < 0.1 && Math.abs(a[1] - 24.0) < 0.1,
     `both calves ≈ 24° at the standing pose (got ${a[0].toFixed(1)}, ${a[1].toFixed(1)})`);
  ok(Math.abs(a[0] - a[1]) < 1e-6, 'symmetric pose: the two calf angles are equal');

  const S = springSumCos2(ZERO, null, 99);
  ok(Math.abs(S - 1.6687) < 1e-3, `S = Σcos²θ ≈ 1.6687 (got ${S.toFixed(4)})`);
}

console.log('Pose dependence:');
{
  // Reaching the foot further out (bigger y) swings the calves more vertical → S rises toward 2.
  const sNear = springSumCos2(ZERO, null, 0);
  const sFar = springSumCos2([0, 140, 73.5], null, 0);
  ok(sFar > sNear, 'a more-extended leg has a larger S (calves nearer vertical)');

  // An unreachable target returns the fallback.
  ok(springSumCos2([0, 5, 73.5], null, 1.6687) === 1.6687, 'unreachable target → fallback S');
}

console.log(failures === 0 ? '\nALL CALF-GEOMETRY TESTS PASSED' : `\n${failures} TEST(S) FAILED`);
process.exit(failures === 0 ? 0 : 1);
