// Tests for the spring-sizing maths. Pure JS (no WASM). Run: node sim/test/spring.test.mjs

import { springWindow } from '../web/spring_sizing.js';

let failures = 0;
const ok = (c, m) => (c ? console.log('  ok:', m) : (console.error('  FAIL:', m), failures++));

const base = {
  mass: 3, cogHeight: 180, footSeparation: 221, calfAngleDeg: 24.0, snr: 3,
  slopeMinDeg: 3, slopeMaxDeg: 20, contactForceN: 4, descentSpeedMmS: 150,
  detectTicks: 2, dtS: 0.02, feet: 4, springTravelMm: 16, preloadMm: 2.0,
  calA: 6.6e6, calB: 5.0, adcNoiseCounts: 3, zeroLoadDistMm: 17.0,
};

console.log('Spring sizing:');
{
  const r = springWindow(base);
  ok(r.feasible && r.kSpringMin < r.kSpringMax,
     `default geometry has a usable window (${r.kSpringMin.toFixed(2)}–${r.kSpringMax.toFixed(2)} N/mm)`);
  ok(r.kSpringRec > r.kSpringMin && r.kSpringRec < r.kSpringMax, 'recommended kSpring is inside the window');
  ok(r.W > 25 && r.W < 35, 'weight ≈ mass·g');
  ok(Math.abs(r.S - 1.6687) < 1e-3, 'S = 2cos²(24°) ≈ 1.6687');
  ok(Math.abs(r.kFootRec - r.kSpringRec * r.S) < 1e-6, 'k_foot = S·kSpring');
  // The sim's modelled spring (~1.69) should be feasible for these defaults.
  ok(1.69 >= r.kSpringMin && 1.69 <= r.kSpringMax, 'the sim spring rate (1.69 N/mm) is inside the window');
}

console.log('Bounds move the right way:');
{
  const finer = springWindow({ ...base, adcNoiseCounts: 1 });
  ok(finer.kSpringMax > springWindow(base).kSpringMax,
     'a quieter ADC raises k_max (resolves slopes with a stiffer spring)');
  const faster = springWindow({ ...base, descentSpeedMmS: 300 });
  ok(faster.kSpringMin < springWindow(base).kSpringMin,
     'a faster foot descent lowers k_min (easier contact detection)');
  const preloaded = springWindow({ ...base, preloadMm: 5 });
  ok(preloaded.kSpringMin > springWindow(base).kSpringMin,
     'more preload raises k_min (the dead-band delays contact detection)');
}

console.log('Friction / stiction:');
{
  // Evaluate standing compression at a fixed actual spring so the comparison isn't confounded
  // by the recommendation shifting with friction.
  const clean = springWindow({ ...base, frictionN: 0, kSpring: 1.79 });
  const rough = springWindow({ ...base, frictionN: 3, kSpring: 1.79 });
  ok(rough.standingCompressionMm < clean.standingCompressionMm,
     'friction lowers the VISIBLE standing compression (the bench symptom)');
  ok(Math.abs(rough.standingCompressionGrossMm - clean.standingCompressionGrossMm) < 1e-9,
     'gross (frictionless) compression is unchanged — friction only hides it');
  ok(rough.frictionSlopeDeg > clean.frictionSlopeDeg && rough.frictionSlopeDeg > 0,
     'friction raises the slope-sensing floor');
  ok(rough.kSpringMin > clean.kSpringMin, 'friction raises k_min (harder to register contact)');
  // The friction floor is k-independent: a softer spring does not lower it.
  const roughSoft = springWindow({ ...base, frictionN: 3, slopeMinDeg: 1 });
  ok(Math.abs(roughSoft.frictionSlopeDeg - rough.frictionSlopeDeg) < 1e-9,
     'the friction slope floor is independent of the recommended spring');
}

console.log('Geometry projection:');
{
  // Larger calf angle → more horizontal calves → smaller Σcos²θ → softer per-foot vertical rate
  // for the same physical spring.
  const steep = springWindow({ ...base, calfAngleDeg: 40 });
  ok(steep.S < springWindow(base).S, 'a larger calf angle lowers S (softer per-foot vertical rate)');
  ok(Math.abs(steep.kFootRec - steep.kSpringRec * steep.S) < 1e-6, 'k_foot = S·kSpring holds at any angle');
}

console.log('Degenerate cases:');
{
  // A very noisy ADC collapses the window (can't resolve slope and detect contact), even after
  // the sensor-resolution self-coupling (softer spring sits deeper where the hall is finer).
  const r = springWindow({ ...base, adcNoiseCounts: 300 });
  ok(!r.feasible && r.kSpringRec === null, 'a too-noisy ADC reports NO window');
  // Preload swallowing the whole contact margin → no contact possible.
  const big = springWindow({ ...base, preloadMm: 20 });
  ok(!big.feasible, 'preload beyond the contact margin reports NO window');
}

console.log(failures === 0 ? '\nALL SPRING TESTS PASSED' : `\n${failures} TEST(S) FAILED`);
process.exit(failures === 0 ? 0 : 1);
