// Browser entry point: load the WASM control core, build the reduced-physics World, wire
// keyboard/controls + transport + logging, and run the render loop. The robot you drive
// here is the real firmware logic (compiled to WASM) — not a reimplementation.

import KodaCore from './koda-core.mjs';
import { World, BTN } from './world.js';
import { drawSide, drawTop, readout } from './render.js';
import { Recorder, diffEvents } from './logger.js';
import { report, springWindow } from './spring_sizing.js';

const M = await KodaCore();

const sideCtx = document.getElementById('side').getContext('2d');
const topCtx = document.getElementById('top').getContext('2d');
const hud = document.getElementById('hud');
const eventsEl = document.getElementById('events');
const $ = (id) => document.getElementById(id);

// ── Terrain options ───────────────────────────────────────────────────────────────────
function makeTerrain() {
  const kind = $('terrain').value;
  const slope = (parseFloat($('slope').value) || 0) * Math.PI / 180;
  if (kind === 'slope') return (x) => x * Math.tan(slope);
  if (kind === 'step') return (x) => (x > 120 ? 35 : 0);          // a raised platform ahead (early contact)
  if (kind === 'stepdown') return (x) => (x > 120 ? -25 : 0);     // ground drops away (late contact)
  if (kind === 'bumps') return (x) => 14 * Math.sin(x / 120);
  return () => 0;                                                 // flat
}

let world = new World(M, { terrain: makeTerrain() });

// ── Step-geometry tuning ──────────────────────────────────────────────────────────────
const GAIT_INPUTS = { stanceY: 'stanceY', stanceZ: 'stanceZ', stepHeight: 'stepHeight', stepLen: 'stepLen' };
function initGaitInputs() {
  const g = world.gaitParams;
  $('stanceY').value = g.stanceY.toFixed(0);
  $('stanceZ').value = g.stanceZ.toFixed(0);
  $('stepHeight').value = g.stepHeight.toFixed(0);
  $('stepLen').value = g.stepLen.toFixed(0);
}
function applyGaitInputs() {
  world.setGaitParams({
    stanceY: parseFloat($('stanceY').value),
    stanceZ: parseFloat($('stanceZ').value),
    stepHeight: parseFloat($('stepHeight').value),
    stepLen: parseFloat($('stepLen').value),
  });
}

// ── Logging ───────────────────────────────────────────────────────────────────────────
const rec = new Recorder();
let prevSnap = null;

function logEvents(st) {
  const ev = diffEvents(prevSnap, st);
  prevSnap = st;
  if (!ev.length) return;
  for (const e of ev) {
    const t = rec.t.toFixed(2).padStart(6);
    const line = `${t}s  ${e}`;
    if ($('consoleLog').checked) console.log('[koda]', line);
    const div = document.createElement('div');
    div.textContent = line;
    eventsEl.prepend(div);
  }
  while (eventsEl.childNodes.length > 200) eventsEl.lastChild.remove();
}

function download(name, text, mime) {
  const url = URL.createObjectURL(new Blob([text], { type: mime }));
  const a = document.createElement('a');
  a.href = url; a.download = name; a.click();
  URL.revokeObjectURL(url);
}

// ── Input state ───────────────────────────────────────────────────────────────────────
const held = {};
let pendingButtons = 0;                         // one-shot edge buttons for the next step
const onKey = (down) => (e) => {
  const k = e.key.toLowerCase();
  held[k] = down;
  if (down) {
    if (k === ' ') { e.preventDefault(); togglePlay(); }
    if (k === 'b') pendingButtons |= BTN.STAND;
    if (k === 'x') pendingButtons |= BTN.SIT;
    if (k === 'g') pendingButtons |= BTN.GAIT;
  }
};
window.addEventListener('keydown', onKey(true));
window.addEventListener('keyup', onKey(false));

const axis = (neg, pos) => (held[neg] ? -1 : 0) + (held[pos] ? 1 : 0);
function command() {
  const cmd = {
    vx: axis('s', 'w') || axis('arrowdown', 'arrowup'),
    vy: axis('a', 'd') || axis('arrowleft', 'arrowright'),
    yaw: axis('q', 'e'),
    height: axis('f', 'r'),
    buttons: pendingButtons,
  };
  pendingButtons = 0;
  return cmd;
}

// ── Transport ─────────────────────────────────────────────────────────────────────────
// Fixed-timestep accumulator: the control core + physics always advance by FIXED_DT per
// tick (so behaviour is identical at any playback speed and a stall can't blow up the
// physics with a huge dt). The speed multiplier just changes how many ticks run per second
// of wall-clock — so 0.1× is true slow-motion, not a different simulation.
const FIXED_DT = 0.02;                  // 50 Hz control tick, matches the firmware
let playing = true, last = 0, acc = 0;
function togglePlay() { playing = !playing; $('play').textContent = playing ? '⏸ Pause' : '▶ Play'; }
function speed() { return parseFloat($('speed').value) || 1; }

function tick() {
  world.step(command(), FIXED_DT);
  const st = world.state();
  if ($('record').checked) rec.push(st, FIXED_DT);
  logEvents(st);
  return st;
}

function draw(st) {
  drawSide(sideCtx, world, st, world.terrain);
  drawTop(topCtx, world, st);
  hud.textContent = readout(st) + `\nrecorded: ${rec.length} ticks · ${speed()}× speed`;
}

function frame(ts) {
  const wallDt = last ? Math.min(0.1, (ts - last) / 1000) : 0;
  last = ts;
  try {
    let st = null;
    if (playing) {
      acc += wallDt * speed();
      let n = 0;
      while (acc >= FIXED_DT && n < 8) { st = tick(); acc -= FIXED_DT; n++; }  // cap catch-up
    }
    if (st) draw(st);
  } catch (e) {
    // A transient error must not kill the animation loop (the old "gets stuck, needs a
    // refresh" symptom). Log it and keep going.
    console.error('[koda] tick error:', e);
  }
  requestAnimationFrame(frame);
}

function rebuildWorld() {
  world = new World(M, {
    terrain: makeTerrain(),
    weightN: parseFloat($('weight').value) || 30,
    useImu: $('imu').checked,
  });
  world.core.setAutoSlope($('auto').checked);
  applyGaitInputs();          // keep the tuned step geometry across a reset
  prevSnap = null;
  draw(world.state());
}

// ── Spring advisor (live spring-rate sizing for the current settings) ─────────────────
function updateAdvisor() {
  const g = world.gaitParams;
  const params = {
    mass: world.weightN / 9.81,
    cogHeight: g.stanceY + world.comHeight,        // COG above the feet
    footSeparation: M.LEG_X_SEPARATION_MM,
    sensorResolutionMm: parseFloat($('sensorRes').value) || 0.05,
    contactForceN: 4,                              // config CONTACT_FORCE_N
    descentSpeedMmS: g.stepHeight * 4,             // ~ step height × cadence
    springTravelMm: world.maxCompMm,
  };
  const r = springWindow(params);
  const kNow = world.springRate;
  const inWindow = r.feasible && kNow >= r.kMin && kNow <= r.kMax;
  $('advisor').textContent = report(params) +
    `\ncurrent spring: ${kNow.toFixed(2)} N/mm — ${r.feasible ? (inWindow ? 'inside the window ✓' : 'OUTSIDE the window ✗') : 'no window'}`;
}

// ── Wire controls ─────────────────────────────────────────────────────────────────────
$('play').addEventListener('click', togglePlay);
$('reset').addEventListener('click', rebuildWorld);
$('step').addEventListener('click', () => draw(tick()));
$('stand').addEventListener('click', () => { pendingButtons |= BTN.STAND; });
$('gait').addEventListener('click', () => { pendingButtons |= BTN.GAIT; });
$('dlCsv').addEventListener('click', () => download('koda-trace.csv', rec.toCSV(), 'text/csv'));
$('dlJson').addEventListener('click', () => download('koda-trace.json', rec.toJSON(), 'application/json'));
$('clearRec').addEventListener('click', () => { rec.clear(); eventsEl.innerHTML = ''; });
for (const id of ['terrain', 'slope', 'weight', 'imu']) $(id).addEventListener('change', rebuildWorld);
$('auto').addEventListener('change', () => world.core.setAutoSlope($('auto').checked));
for (const id of Object.values(GAIT_INPUTS)) $(id).addEventListener('input', () => { applyGaitInputs(); updateAdvisor(); });
for (const id of ['sensorRes', 'weight']) $(id).addEventListener('input', updateAdvisor);

initGaitInputs();
updateAdvisor();
requestAnimationFrame(frame);
