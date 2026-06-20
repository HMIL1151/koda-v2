// Headless regression tests for the REAL control core (the WASM build of the firmware C++).
// Run:  node sim/test/core.test.mjs   (after sim/wasm/build.sh)
//
// These exercise the actual shipping logic — no Python/JS reimplementation — so they can't
// drift from what runs on the ESP32. Stage 1 coverage: IK, the hall model, the state
// machine, gait motion, early-ground-contact, and the incline balance response.

import KodaCore from '../web/koda-core.mjs';

const M = await KodaCore();

let failures = 0;
function ok(cond, msg) {
  if (cond) { console.log('  ok:', msg); }
  else { console.error('  FAIL:', msg); failures++; }
}

// Button bitmask (mirrors bindings.cpp Buttons).
const BTN = { STAND: 1, SIT: 2, GAIT: 4 };

// Convenience wrapper around SimCore.step (forces default to none).
function makeStepper(core) {
  return (o = {}) => {
    const f = o.forces ?? [0, 0, 0, 0];
    return core.step(o.vx ?? 0, o.vy ?? 0, o.yaw ?? 0, o.height ?? 0, o.buttons ?? 0,
                     f[0], f[1], f[2], f[3], o.pitch ?? 0, o.roll ?? 0, o.dt ?? 0.02);
  };
}

// ── 1. Inverse kinematics ────────────────────────────────────────────────────────────
console.log('Inverse kinematics:');
{
  const neutral = M.inverseKinematics(M.ZERO_X, M.ZERO_Y, M.ZERO_Z);
  ok(neutral[0] === 1, 'neutral foot target is solvable');
  ok(neutral.slice(1).every(Number.isFinite), 'neutral joint angles are finite');
  const reach = M.inverseKinematics(30, 125, 73.5);   // a reachable offset target
  ok(reach[0] === 1, 'an offset foot target is solvable');
}

// ── 2. Hall force model invariants (s0→0 N, s1→full-load force, any K) ─────────────────
console.log('Hall model:');
for (const [s0, s1, label] of [[5000, 20000, 'rising'], [22000, 8000, 'falling']]) {
  const K = M.solveHallK(s0, s1);
  ok(Math.abs(M.hallForceFromSignal(s0, s0, s1, K)) < 1e-2, `${label}: unloaded → 0 N`);
  ok(Math.abs(M.hallForceFromSignal(s1, s0, s1, K) - M.HALL_FULL_LOAD_FORCE_N) < 1e-2,
     `${label}: full load → ${M.HALL_FULL_LOAD_FORCE_N} N`);
}

// ── 3. State machine: SLEEP → STAND → WALK → STAND → SIT ───────────────────────────────
console.log('State machine:');
{
  const core = new M.SimCore();
  const step = makeStepper(core);
  const SLEEP = 0, STAND = 1, WALK = 2, SIT = 3;
  step();                              ok(core.state() === SLEEP, 'boots in SLEEP');
  step({ buttons: BTN.STAND });        ok(core.state() === STAND, 'stand toggle → STAND');
  for (let i = 0; i < 3; i++) step({ vx: 0.8 });
                                       ok(core.state() === WALK, 'stick input → WALK');
  for (let i = 0; i < 120; i++) step({ vx: 0 });   // graceful stop winds down over ~1 cycle
                                       ok(core.state() === STAND, 'idle → graceful stop → STAND');
  step({ buttons: BTN.SIT });          ok(core.state() === SIT, 'sit toggle → SIT');
  core.delete();
}

// ── 4. Gait produces motion + phase advances ──────────────────────────────────────────
console.log('Gait:');
{
  const core = new M.SimCore();
  const step = makeStepper(core);
  step({ buttons: BTN.STAND });
  const a = step({ vx: 0.9 });
  let moved = false, p0 = core.cyclePhase();
  for (let i = 0; i < 25; i++) {
    const b = step({ vx: 0.9 });
    if (b.some((v, j) => Math.abs(v - a[j]) > 1)) moved = true;
  }
  ok(moved, 'walking changes servo angles over time');
  ok(core.cyclePhase() !== p0, 'cycle phase advances while walking');
  ok(Math.abs(core.swingFraction() - 0.5) < 1e-6, 'trot swing fraction is 0.5');
  core.delete();
}

// ── 5. Early ground contact: load a foot mid-swing → flag fires ────────────────────────
console.log('Early ground contact:');
{
  const core = new M.SimCore();
  const step = makeStepper(core);
  step({ buttons: BTN.STAND });
  // Walk until leg 0 (FL) is mid-swing (past the min trusted swing fraction).
  let found = false;
  for (let i = 0; i < 200 && !found; i++) {
    step({ vx: 0.9 });
    const lp = core.localPhase();
    const sf = core.swingFraction();
    if (lp[0] < sf && lp[0] / sf > 0.4) {
      // Next step: inject a big force on FL while it's still mid-swing.
      step({ vx: 0.9, forces: [50, 0, 0, 0] });
      found = core.earlyContact(0);
    }
  }
  ok(found, 'a foot loaded mid-swing raises its early-contact flag');
  core.delete();
}

// ── 6. Incline balance responds to asymmetric load ────────────────────────────────────
console.log('Balance:');
{
  const settle = (forces) => {
    const core = new M.SimCore();
    const step = makeStepper(core);
    step({ buttons: BTN.STAND });
    let s;
    for (let i = 0; i < 60; i++) s = step({ forces });   // let the EMA settle
    core.delete();
    return s;
  };
  const balanced  = settle([20, 20, 20, 20]);
  const rearHeavy = settle([10, 10, 30, 30]);   // rear loaded → COG should shift forward
  const frontHeavy = settle([30, 30, 10, 10]);

  const diff = (a, b) => a.reduce((s, v, i) => s + Math.abs(v - b[i]), 0);
  ok(diff(rearHeavy, balanced) > 1, 'rear-heavy load shifts the pose');
  ok(diff(frontHeavy, balanced) > 1, 'front-heavy load shifts the pose');
  // Opposite imbalances must push the pose in opposite directions (sign-consistent).
  const opposite = balanced.some((v, i) =>
    (rearHeavy[i] - v) * (frontHeavy[i] - v) < -1e-3);
  ok(opposite, 'opposite imbalances move the pose in opposite directions');
}

// ── 6a. Slope estimation from the calf springs (static, unbalanced reference stance) ──
console.log('Slope estimation:');
{
  const W = 30, h = 180, halfX = 110, halfZ = 73.5;
  // Forces for a fore/aft slope: COG offset d = h·sin θ → front/rear load split dF = W·d/halfX.
  const forcesForSlope = (deg) => {
    const d = h * Math.sin(deg * Math.PI / 180);
    const dF = W * d / halfX;
    const Rf = (W + dF) / 2, Rr = (W - dF) / 2;
    return [Rf / 2, Rf / 2, Rr / 2, Rr / 2];   // FL FR (front), RR RL (rear)
  };
  const flat = M.estimateSlope(...forcesForSlope(0), halfX, halfZ, h);
  ok(Math.abs(flat[0]) < 0.5 * Math.PI / 180, 'flat ground → ~0° estimated pitch');
  for (const deg of [8, 16, -12]) {
    const [pitch] = M.estimateSlope(...forcesForSlope(deg), halfX, halfZ, h);
    ok(Math.abs(pitch * 57.2958 - deg) < 2, `slope ${deg}° → estimated within 2° (${(pitch * 57.2958).toFixed(1)}°)`);
  }
  // A roll: more load on the right feet → positive roll estimate.
  const rolled = M.estimateSlope(5, 10, 10, 5, halfX, halfZ, h);  // FR,RR (right) heavier
  ok(rolled[1] > 0, 'right-heavy load → positive roll estimate');
}

// ── 6b. Late ground contact: stance foot with no load reaches down for the ground ────
console.log('Late ground contact:');
{
  const core = new M.SimCore();
  const drive = (f) => core.step(0, 0, 0, 0, 0, f[0], f[1], f[2], f[3], 0, 0, 0.02);
  core.step(0, 0, 0, 0, BTN.STAND, 0, 0, 0, 0, 0, 0, 0.02);
  for (let i = 0; i < 30; i++) drive([10, 10, 10, 10]);        // standing, all feet loaded
  const y0 = core.footTarget(0)[1];
  ok(!core.lateContact(0), 'a loaded stance foot is not flagged late');

  let yReach = y0;
  for (let i = 0; i < 50; i++) { drive([0, 10, 10, 10]); yReach = core.footTarget(0)[1]; }  // FL loses ground
  ok(core.lateContact(0), 'a stance foot with no load flags late contact');
  ok(yReach > y0 + 10, 'late contact reaches the foot down to find ground');

  for (let i = 0; i < 10; i++) drive([10, 10, 10, 10]);        // ground found again
  ok(!core.lateContact(0), 'late clears once the foot is loaded again');
  core.delete();
}

// ── 7. Tunable step geometry + reachability flag ─────────────────────────────────────
console.log('Tunable geometry & reachability:');
{
  const core = new M.SimCore();
  const step = makeStepper(core);
  // Keep the feet loaded so the late-contact ground-probe doesn't move the targets here.
  const settle = (n = 12) => { for (let i = 0; i < n; i++) step({ forces: [10, 10, 10, 10] }); };
  step({ buttons: BTN.STAND });
  settle();

  const y0 = core.footTarget(0)[1];
  core.setGaitParams(0, 100, 73.5, 40, 45);        // lower (crouched) stance — well within reach
  settle();
  ok(Math.abs(core.footTarget(0)[1] - y0) > 20, 'changing stance height moves the neutral foot target');
  ok([0, 1, 2, 3].every((l) => core.ikOk(l)), 'a reachable stance solves on all legs');

  const z0 = core.footTarget(0)[2];
  core.setGaitParams(0, 125, 95, 40, 45);          // wider track
  settle();
  ok(Math.abs(core.footTarget(0)[2] - z0) > 10, 'changing stance width moves the foot laterally');

  core.setGaitParams(0, 400, 73.5, 40, 45);        // beyond leg reach (~150mm)
  settle(5);
  ok([0, 1, 2, 3].every((l) => !core.ikOk(l)), 'an out-of-range stance flags no-IK on every leg');
  core.delete();
}

console.log(failures === 0
  ? '\nALL CORE TESTS PASSED (against the real firmware logic)'
  : `\n${failures} TEST(S) FAILED`);
process.exit(failures === 0 ? 0 : 1);
