// Headless closed-loop tests for the reduced-physics World driving the real control core.
// Run:  node sim/test/world.test.mjs   (after sim/wasm/build.sh)
//
// Validates the sim physics (settle stability, walking progress, standing on an incline)
// without a browser. The control logic exercised is the real firmware WASM core.

import KodaCore from '../web/koda-core.mjs';
import { World, BTN } from '../web/world.js';

const M = await KodaCore();

let failures = 0;
const ok = (c, m) => (c ? console.log('  ok:', m) : (console.error('  FAIL:', m), failures++));
const run = (world, cmd, n) => { for (let i = 0; i < n; i++) world.step(cmd, 0.02); };

// ── Stand on flat ground ───────────────────────────────────────────────────────────
console.log('Stand on flat ground:');
{
  const w = new World(M);
  w.step({ buttons: BTN.STAND }, 0.02);
  run(w, {}, 300);
  const s = w.state();
  ok(Number.isFinite(s.body.y), 'body height stays finite (no divergence)');
  ok(Math.abs(s.totalForce - w.weightN) / w.weightN < 0.2, 'total foot force settles near weight');
  ok(s.contact.every(Boolean), 'all four feet are in contact');
  ok(Math.abs(s.body.pitch) < 0.05 && Math.abs(s.body.roll) < 0.05, 'body stays level on flat');
}

// ── Walk forward ─────────────────────────────────────────────────────────────────────
console.log('Walk forward:');
{
  const w = new World(M);
  w.step({ buttons: BTN.STAND }, 0.02);
  run(w, {}, 60);
  const x0 = w.state().body.x;
  run(w, { vx: 0.9 }, 300);
  const s = w.state();
  ok(s.robotState === 2, 'robot is in WALK');
  ok(s.body.x - x0 > 50, 'body advances forward while walking');
  ok(Number.isFinite(s.body.y), 'stays stable while walking');
}

// ── Stand on an incline ──────────────────────────────────────────────────────────────
console.log('Stand on an incline:');
{
  const slope = 12 * Math.PI / 180;                 // 12° ground slope along x
  const w = new World(M, { terrain: (x) => x * Math.tan(slope) });
  w.step({ buttons: BTN.STAND }, 0.02);
  run(w, {}, 600);
  const s = w.state();
  ok(Number.isFinite(s.body.y), 'stays stable on the slope (no divergence)');
  ok(s.contact.every(Boolean), 'all feet keep contact on the slope');
  ok(Math.abs(s.totalForce - w.weightN) / w.weightN < 0.25, 'still supports its weight on the slope');
  // The force re-centring should not leave the load wildly lopsided front/rear.
  const front = s.forces[0] + s.forces[1], rear = s.forces[2] + s.forces[3];
  ok(Math.min(front, rear) / Math.max(front, rear) > 0.6, 'load is well balanced front/rear');
  ok(Math.abs(s.cog.x - s.centroid.x) < 5, 'COG sits over the support centroid (statically)');
}

// ── Walk on an incline — automatic stop → measure → adjust → resume ──────────────────
console.log('Walk on an incline (auto stop-measure-resume):');
{
  const slope = 10 * Math.PI / 180;                         // gentle enough to fully re-centre
  const w = new World(M, { terrain: (x) => x * Math.tan(slope) });
  w.step({ buttons: BTN.STAND }, 0.02);
  run(w, {}, 60);

  // Walking onto the slope: the contact pattern flags a slope → the robot auto-stops,
  // measures it from the springs, and resumes (driving forward throughout).
  const seen = new Set();
  for (let i = 0; i < 900; i++) { w.step({ vx: 0.85 }, 0.02); seen.add(w.state().robotState); }
  ok(seen.has(5), 'auto cycle enters MEASURING');
  ok(Math.abs(w.core.measuredPitch()) > 3 * Math.PI / 180, 'measures a non-zero slope from the springs');
  ok(w.state().robotState === 2, 'auto-resumes walking');

  // After measuring, the feed-forward bias holds the COG over the support while walking.
  let sum = 0, n = 0;
  for (let i = 0; i < 400; i++) {
    w.step({ vx: 0.85 }, 0.02);
    const st = w.state();
    if (st.centroid) { sum += Math.abs(st.cog.x - st.centroid.x); n++; }
  }
  ok(n > 0 && sum / n < 18, `COG re-centred walking uphill after measuring (${(sum / n).toFixed(1)}mm)`);
  ok(Number.isFinite(w.state().body.y), 'stays stable walking on the slope');
}

// ── The auto cycle fires on slopes, not on flat ground ───────────────────────────────
console.log('Auto cycle discrimination:');
{
  const flat = new World(M);
  flat.step({ buttons: BTN.STAND }, 0.02);
  run(flat, {}, 60);
  let measured = false;
  for (let i = 0; i < 900; i++) { flat.step({ vx: 0.85 }, 0.02); if (flat.state().robotState === 5) measured = true; }
  ok(!measured, 'flat ground never triggers the measure cycle');
}

// ── Graceful start + stop (no jump in the control's foot targets either way) ─────────
console.log('Graceful start/stop:');
{
  // Largest per-tick change in a foot TARGET (leg frame) — this is the control output, so
  // it isolates a control snap from the sim's body motion.
  const maxTargetStep = (w, cmd, n, track) => {
    let prev = w.state().footTargets.map((t) => [...t]);
    let mx = 0;
    for (let i = 0; i < n; i++) {
      w.step(cmd, 0.02);
      const t = w.state().footTargets;
      if (track) for (let l = 0; l < 4; l++) {
        mx = Math.max(mx, Math.hypot(t[l][0] - prev[l][0], t[l][1] - prev[l][1], t[l][2] - prev[l][2]));
      }
      prev = t.map((q) => [...q]);
    }
    return mx;
  };

  const w = new World(M);
  w.step({ buttons: BTN.STAND }, 0.02);
  run(w, {}, 60);

  const startMax = maxTargetStep(w, { vx: 0.9 }, 25, true);   // first ~half cycle of walking
  const walkMax = maxTargetStep(w, { vx: 0.9 }, 80, true);    // steady-state reference
  const states = new Set();
  let stopMax = 0;
  let prev = w.state().footTargets.map((t) => [...t]);
  for (let i = 0; i < 160; i++) {
    w.step({ vx: 0 }, 0.02);
    const st = w.state();
    states.add(st.robotState);
    for (let l = 0; l < 4; l++) {
      const t = st.footTargets;
      stopMax = Math.max(stopMax, Math.hypot(t[l][0] - prev[l][0], t[l][1] - prev[l][1], t[l][2] - prev[l][2]));
    }
    prev = st.footTargets.map((q) => [...q]);
  }

  // ≤1.3× the steady-walk step proves no snap (real snaps were 4–8×); the small margin
  // absorbs trajectory variation from the feed-forward balance split.
  ok(startMax <= walkMax * 1.3, `smooth start: no target jump (start ${startMax.toFixed(1)} ≤ 1.3× walk ${walkMax.toFixed(1)}mm)`);
  ok(states.has(4), 'stop passes through the STOPPING state');
  ok(w.state().robotState === 1, 'ends in STAND');
  ok(stopMax <= walkMax * 1.3, `smooth stop: no target jump (stop ${stopMax.toFixed(1)} ≤ 1.3× walk ${walkMax.toFixed(1)}mm)`);
}

// ── No foot dragging when starting to walk (planted feet stay put in the world) ──────
console.log('No-drag start:');
{
  // Largest world-frame move of a foot that is in contact on consecutive ticks — a
  // planted foot should barely move; gross movement is the foot skating on the floor.
  const plantedDrag = (w, cmd, n) => {
    let prevF = w.state().feet.map((p) => ({ ...p }));
    let prevC = w.state().contact.slice();
    let mx = 0;
    for (let i = 0; i < n; i++) {
      w.step(cmd, 0.02);
      const st = w.state();
      for (let l = 0; l < 4; l++) {
        if (prevC[l] && st.contact[l]) {
          mx = Math.max(mx, Math.hypot(st.feet[l].x - prevF[l].x, st.feet[l].z - prevF[l].z));
        }
      }
      prevF = st.feet.map((p) => ({ ...p }));
      prevC = st.contact.slice();
    }
    return mx;
  };

  const w = new World(M);
  w.step({ buttons: BTN.STAND }, 0.02);
  run(w, {}, 80);
  const startDrag = plantedDrag(w, { vx: 0.9 }, 120);   // includes the start transient
  const steadyDrag = plantedDrag(w, { vx: 0.9 }, 200);  // steady-state reference
  ok(startDrag <= steadyDrag * 3, `start doesn't drag planted feet (start ${startDrag.toFixed(1)} ≤ 3× steady ${steadyDrag.toFixed(1)}mm/tick)`);
}

// ── Walk into a step-up → early ground contact fires ─────────────────────────────────
console.log('Early ground contact on a step-up:');
{
  const w = new World(M, { terrain: (x) => (x > 120 ? 35 : 0) });  // raised platform ahead
  w.step({ buttons: BTN.STAND }, 0.02);
  run(w, {}, 60);
  let sawEarly = false;
  for (let i = 0; i < 800 && !sawEarly; i++) {
    w.step({ vx: 0.9 }, 0.02);                                       // walk toward the step
    if (w.state().early.some(Boolean)) sawEarly = true;
  }
  ok(sawEarly, 'a swinging foot meeting the step raises early-contact');
  ok(Number.isFinite(w.state().body.y), 'stays stable approaching the step');
}

// ── Walk down a step → late contact, leg reaches down, no fall-through ────────────────
console.log('Late ground contact on a step-down:');
{
  const w = new World(M, { terrain: (x) => (x > 120 ? -20 : 0) });   // ground drops away ahead
  w.step({ buttons: BTN.STAND }, 0.02);
  run(w, {}, 60);
  let sawLate = false, finite = true;
  for (let i = 0; i < 600; i++) {
    w.step({ vx: 0.85 }, 0.02);
    const st = w.state();
    if (st.late.some(Boolean)) sawLate = true;
    if (!Number.isFinite(st.body.y)) finite = false;
  }
  const s = w.state();
  ok(sawLate, 'a stance foot over the drop raises late-contact');
  ok(finite && Number.isFinite(s.body.y), 'stays stable crossing the drop (no fall-through)');
  ok(s.contact.filter(Boolean).length >= 2, 'regains solid support on the lower ground');
}

// ── Turning in place doesn't drag planted feet or blow up the body ───────────────────
console.log('Turn-in-place stability:');
{
  const w = new World(M);
  w.step({ buttons: BTN.STAND }, 0.02);
  run(w, {}, 80);

  // Straight-walk references: steady planted-foot drag and peak total foot force. Gait force
  // naturally peaks above the static weight during the double-support overlap, so the turn is
  // judged against straight walking — not against a fixed fraction of the weight.
  const walkProbe = (cmd, n) => {
    let prevF = w.state().feet.map((p) => ({ ...p }));
    let prevC = w.state().contact.slice();
    let drag = 0, peakForce = 0;
    for (let i = 0; i < n; i++) {
      w.step(cmd, 0.02);
      const st = w.state();
      for (let l = 0; l < 4; l++) {
        if (prevC[l] && st.contact[l]) drag = Math.max(drag, Math.hypot(st.feet[l].x - prevF[l].x, st.feet[l].z - prevF[l].z));
      }
      peakForce = Math.max(peakForce, st.totalForce);
      prevF = st.feet.map((p) => ({ ...p }));
      prevC = st.contact.slice();
    }
    return { drag, peakForce };
  };
  const { drag: steadyDrag, peakForce: walkForce } = walkProbe({ vx: 0.9 }, 200);

  // Now turn in place (gait running, yaw held) and watch for divergence.
  w.step({ buttons: BTN.GAIT }, 0.02);
  let prevF = w.state().feet.map((p) => ({ ...p }));
  let prevC = w.state().contact.slice();
  let turnDrag = 0, maxTilt = 0, maxForce = 0, finite = true;
  for (let i = 0; i < 200; i++) {
    w.step({ yaw: 1 }, 0.02);
    const st = w.state();
    for (let l = 0; l < 4; l++) {
      if (prevC[l] && st.contact[l]) turnDrag = Math.max(turnDrag, Math.hypot(st.feet[l].x - prevF[l].x, st.feet[l].z - prevF[l].z));
    }
    maxTilt = Math.max(maxTilt, Math.abs(st.body.pitch), Math.abs(st.body.roll));
    maxForce = Math.max(maxForce, st.totalForce);
    if (!Number.isFinite(st.body.y)) finite = false;
    prevF = st.feet.map((p) => ({ ...p }));
    prevC = st.contact.slice();
  }
  const s = w.state();
  ok(Math.abs(s.body.yaw) > 1.0, 'the robot actually turns (body yaw accumulates)');
  ok(turnDrag <= steadyDrag * 3, `turning doesn't drag planted feet (turn ${turnDrag.toFixed(1)} ≤ 3× steady ${steadyDrag.toFixed(1)}mm/tick)`);
  ok(maxTilt < 0.3, `body stays level while turning (max tilt ${maxTilt.toFixed(2)} rad, well under the 0.5 clamp)`);
  ok(finite && Math.abs(s.body.y - w.gaitParams.stanceY) < 25, 'body height stays near stance height (no ballooning)');
  ok(maxForce <= walkForce * 1.2, `foot force stays sane while turning (turn ${maxForce.toFixed(0)}N ≤ 1.2× straight-walk peak ${walkForce.toFixed(0)}N)`);
}

console.log(failures === 0 ? '\nALL WORLD TESTS PASSED' : `\n${failures} TEST(S) FAILED`);
process.exit(failures === 0 ? 0 : 1);
