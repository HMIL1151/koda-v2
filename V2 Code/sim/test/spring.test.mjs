// Tests for the spring-sizing maths. Pure JS (no WASM). Run: node sim/test/spring.test.mjs

import { springWindow } from '../web/spring_sizing.js';

let failures = 0;
const ok = (c, m) => (c ? console.log('  ok:', m) : (console.error('  FAIL:', m), failures++));

const base = {
  mass: 3, cogHeight: 180, footSeparation: 221, sensorResolutionMm: 0.05, snr: 3,
  slopeMinDeg: 3, slopeMaxDeg: 20, contactForceN: 4, descentSpeedMmS: 150,
  detectTicks: 2, dtS: 0.02, stanceFeet: 2, springTravelMm: 16,
};

console.log('Spring sizing:');
{
  const r = springWindow(base);
  ok(r.feasible && r.kMin < r.kMax, `default geometry has a usable window (${r.kMin.toFixed(2)}–${r.kMax.toFixed(2)} N/mm)`);
  ok(r.kRec > r.kMin && r.kRec < r.kMax, 'recommended k is inside the window');
  ok(r.W > 25 && r.W < 35, 'weight ≈ mass·g');
  // The sim's modelled spring (~1.69) should be feasible for these defaults.
  ok(1.69 >= r.kMin && 1.69 <= r.kMax, 'the sim spring rate (1.69 N/mm) is inside the window');
}

console.log('Bounds move the right way:');
{
  const finer = springWindow({ ...base, sensorResolutionMm: 0.02 });
  ok(finer.kMax > springWindow(base).kMax, 'a finer sensor raises k_max (resolves slopes with a stiffer spring)');
  const faster = springWindow({ ...base, descentSpeedMmS: 300 });
  ok(faster.kMin < springWindow(base).kMin, 'a faster foot descent lowers k_min (easier contact detection)');
}

console.log('Degenerate case:');
{
  // A very coarse sensor collapses the window (can't resolve slope and detect contact).
  const r = springWindow({ ...base, sensorResolutionMm: 0.8 });
  ok(!r.feasible && r.kRec === null, 'a too-coarse sensor reports NO window');
}

console.log(failures === 0 ? '\nALL SPRING TESTS PASSED' : `\n${failures} TEST(S) FAILED`);
process.exit(failures === 0 ? 0 : 1);
