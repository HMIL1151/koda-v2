// Canvas rendering for the simulator: a side view (sagittal, x–y) and a top view (x–z,
// showing the support polygon + centre of gravity). Pure drawing — it reads a World and
// paints; no simulation here.

import { legLinkage } from './leg5bar.js';

const LEG_ORDER = ['FL', 'FR', 'RR', 'RL'];

// Colours
const C = {
  bg: '#11151c', grid: '#1c2330', ground: '#3a4658', body: '#e8eef5',
  legNear: '#7fb2ff', legFar: '#3b5170', contact: '#37d67a', airborne: '#6b7785',
  early: '#ff5d5d', com: '#ffd23f', force: '#37d67a', text: '#aab4c2',
  noIk: '#ff3df0',   // magenta: foot target unreachable / out of range (no IK solution)
  late: '#ffa53d',   // orange: stance foot found no ground, reaching down to find it
};

function clear(ctx, w, h) {
  ctx.fillStyle = C.bg;
  ctx.fillRect(0, 0, w, h);
}

function dot(ctx, x, y, r, fill) {
  ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.fillStyle = fill; ctx.fill();
}

function line(ctx, x1, y1, x2, y2, stroke, width = 2) {
  ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2);
  ctx.strokeStyle = stroke; ctx.lineWidth = width; ctx.stroke();
}

function footColour(st, leg) {
  if (st.ikOk && !st.ikOk[leg]) return C.noIk;            // unreachable target
  if (st.early[leg]) return C.early;                       // hit ground early (terrain up)
  if (st.late && st.late[leg]) return C.late;              // no ground in stance (terrain down)
  return st.contact[leg] ? C.contact : C.airborne;
}

// Hip-local sagittal point (lx fore/aft, lyDown) → world (x,y), rotated by body pitch.
// Matches world.js footWorld's x-y projection, so the linkage foot coincides with the foot.
function legPoint(hip, lx, lyDown, pitch) {
  const c = Math.cos(pitch), s = Math.sin(pitch);
  return { x: hip.x + lx * c + lyDown * s, y: hip.y + lx * s - lyDown * c };
}

// Draw one leg as the real 5-bar linkage: two knee servos at the hip, two thighs, two
// knees, two calves to the shared foot — plus the foot dot and a force bar.
function drawLeg5bar(ctx, world, st, leg, colour, kneeColour, sx, sy) {
  const hip = world.hipWorld(leg);
  const ft = st.footTargets[leg];                 // [x fwd, y down, z]
  const link = legLinkage(ft[0], ft[1], world.M);
  const pitch = st.body.pitch;
  if (st.ikOk && !st.ikOk[leg]) { colour = C.noIk; kneeColour = C.noIk; }  // unreachable

  if (!link) {                                    // unreachable → fall back to a line
    const foot = st.feet[leg];
    if (foot) line(ctx, sx(hip.x), sy(hip.y), sx(foot.x), sy(foot.y), colour, 2);
    return;
  }

  const P = (p) => { const w = legPoint(hip, p.x, p.y, pitch); return [sx(w.x), sy(w.y)]; };
  const [sF, sR, kF, kR, ft2] = [link.servoF, link.servoR, link.kneeF, link.kneeR, link.foot].map(P);

  line(ctx, sF[0], sF[1], sR[0], sR[1], colour, 2);   // base between knee servos
  line(ctx, sF[0], sF[1], kF[0], kF[1], colour, 3);   // front thigh
  line(ctx, sR[0], sR[1], kR[0], kR[1], colour, 3);   // rear thigh
  line(ctx, kF[0], kF[1], ft2[0], ft2[1], colour, 3); // front calf (spring)
  line(ctx, kR[0], kR[1], ft2[0], ft2[1], colour, 3); // rear calf (spring)
  dot(ctx, kF[0], kF[1], 3, kneeColour);
  dot(ctx, kR[0], kR[1], 3, kneeColour);

  const foot = st.feet[leg] || { x: ft[0], y: hip.y };
  dot(ctx, sx(foot.x), sy(foot.y), 5, footColour(st, leg));
  const fbar = Math.min(40, st.forces[leg] * 1.2);    // force bar up from the foot
  line(ctx, sx(foot.x), sy(foot.y), sx(foot.x), sy(foot.y) - fbar, C.force, 4);
}

// ── Side view: world (x,y) → screen, camera follows body.x ────────────────────────────
export function drawSide(ctx, world, st, terrain) {
  const W = ctx.canvas.width, H = ctx.canvas.height;
  clear(ctx, W, H);
  const scale = 0.9;                          // px per mm
  const camX = st.body.x;
  const cy = H * 0.62;
  const sx = (x) => W / 2 + (x - camX) * scale;
  const sy = (y) => cy - y * scale;

  // Terrain profile across the view.
  ctx.beginPath();
  for (let px = 0; px <= W; px += 8) {
    const wx = camX + (px - W / 2) / scale;
    const yy = sy(terrain(wx, 0));
    px === 0 ? ctx.moveTo(px, yy) : ctx.lineTo(px, yy);
  }
  ctx.lineTo(W, H); ctx.lineTo(0, H); ctx.closePath();
  ctx.fillStyle = C.ground; ctx.globalAlpha = 0.5; ctx.fill(); ctx.globalAlpha = 1;

  // Legs: far side (right, z>0) faded, near side (left) bright. Draw far first.
  const draw = (legs, colour, kneeColour) => {
    for (const leg of legs) {
      drawLeg5bar(ctx, world, st, leg, colour, kneeColour, sx, sy);
    }
  };
  draw([1, 2], C.legFar, C.legFar);     // FR, RR
  draw([0, 3], C.legNear, '#cfe3ff');   // FL, RL

  // Body line (front hip → rear hip, near side).
  const hf = world.hipWorld(0), hr = world.hipWorld(3);
  line(ctx, sx(hf.x), sy(hf.y), sx(hr.x), sy(hr.y), C.body, 5);
  dot(ctx, sx(st.body.x), sy(st.body.y), 4, C.com);   // ~CoM at body origin

  label(ctx, 8, 16, `side · pitch ${(st.body.pitch * 57.3).toFixed(1)}°`);
}

// ── Top view: world (x,z) → screen. x(forward)=up, z(right)=right. ────────────────────
export function drawTop(ctx, world, st) {
  const W = ctx.canvas.width, H = ctx.canvas.height;
  clear(ctx, W, H);
  const scale = 0.7;
  const camX = st.body.x;
  const sx = (z) => W / 2 + z * scale;
  const sy = (x) => H / 2 - (x - camX) * scale;

  // Support polygon (feet in contact), FL,FR,RR,RL is already a convex ring order.
  const inC = [0, 1, 2, 3].filter((l) => st.contact[l] && st.feet[l]);
  if (inC.length >= 3) {
    ctx.beginPath();
    inC.forEach((l, i) => {
      const f = st.feet[l];
      i === 0 ? ctx.moveTo(sx(f.z), sy(f.x)) : ctx.lineTo(sx(f.z), sy(f.x));
    });
    ctx.closePath();
    ctx.fillStyle = 'rgba(55,214,122,0.12)'; ctx.fill();
    ctx.strokeStyle = 'rgba(55,214,122,0.5)'; ctx.lineWidth = 1.5; ctx.stroke();
  }

  // Body rectangle (hips).
  ctx.beginPath();
  [0, 1, 2, 3].forEach((l, i) => {
    const h = world.hipWorld(l);
    i === 0 ? ctx.moveTo(sx(h.z), sy(h.x)) : ctx.lineTo(sx(h.z), sy(h.x));
  });
  ctx.closePath(); ctx.strokeStyle = C.body; ctx.lineWidth = 2; ctx.stroke();

  // Feet.
  for (const leg of [0, 1, 2, 3]) {
    const f = st.feet[leg]; if (!f) continue;
    dot(ctx, sx(f.z), sy(f.x), 6, footColour(st, leg));
    label(ctx, sx(f.z) + 8, sy(f.x) - 6, LEG_ORDER[leg]);
  }

  // Support centroid + centre of gravity. The balance controller's job is to keep these
  // together (CoG over the middle of the feet) — on a slope you can watch it converge.
  if (st.centroid) {
    dot(ctx, sx(st.centroid.z), sy(st.centroid.x), 4, '#8b97a8');
    line(ctx, sx(st.centroid.z), sy(st.centroid.x), sx(st.cog.z), sy(st.cog.x), '#8b97a8', 1);
  }
  dot(ctx, sx(st.cog.z), sy(st.cog.x), 6, C.com);
  label(ctx, 8, 16, 'top · CoG (yellow) over support centroid (grey)');
}

function label(ctx, x, y, text) {
  ctx.fillStyle = C.text; ctx.font = '12px ui-monospace, monospace'; ctx.fillText(text, x, y);
}

// Compact text readout for the HUD element.
export function readout(st) {
  const names = ['SLEEP', 'STAND', 'WALK', 'SIT', 'STOPPING', 'MEASURING'];
  const terrain = ['flat', 'uneven', 'slope'][st.terrain] ?? '?';
  const f = st.forces.map((v) => v.toFixed(1)).join('  ');
  const cogOff = st.centroid ? (st.cog.x - st.centroid.x).toFixed(1) : '–';
  return [
    `state: ${names[st.robotState] ?? '?'}    terrain: ${terrain}${st.autoSlope ? '  [auto]' : ''}`,
    `phase: ${st.cyclePhase.toFixed(2)}`,
    `pitch: ${(st.body.pitch * 57.3).toFixed(1)}°  roll: ${(st.body.roll * 57.3).toFixed(1)}°`,
    `slope est (live): ${((st.slopePitch || 0) * 57.3).toFixed(1)}°  measured: ${((st.measuredPitch || 0) * 57.3).toFixed(1)}°`,
    `forces (N) FL FR RR RL: ${f}`,
    `total: ${st.totalForce.toFixed(1)} N`,
    `CoG−centroid: ${cogOff} mm`,
    `early contact: ${st.early.map((e, i) => (e ? LEG_ORDER[i] : '·')).join(' ')}`,
    `late  contact: ${(st.late || []).map((e, i) => (e ? LEG_ORDER[i] : '·')).join(' ')}`,
    st.ikOk && st.ikOk.some((v) => !v)
      ? `⚠ OUT OF RANGE — no IK: ${st.ikOk.map((v, i) => (v ? '·' : LEG_ORDER[i])).join(' ')}`
      : 'reach: all legs OK',
  ].join('\n');
}
