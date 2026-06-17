// gait.js — terrain profiles and a quasi-static walking stride.
//
// Walking is modelled with the front/rear legs as lateral PAIRS. The two legs
// alternate stance and swing: each foot holds its plant while the body advances
// over it, then lifts in an arc and steps forward. Crucially the SOLVE always
// uses each foot's on-ground position (so both contacts stay planted — the
// stable 2-contact equilibrium the solver handles); the vertical lift is a
// drawing-only flourish (drawY), so a swinging leg is never pinned lifted (which
// would make it push on the ground). Kinematic / quasi-static — no momentum.

import { degToRad } from './helpers.js';

// Returns a terrain height function worldX -> worldY (+y is DOWN).
export function makeTerrain(type, p = {}) {
    if (type === 'flat') return () => 0;
    if (type === 'slope') {
        const m = Math.tan(degToRad(p.angleDeg || 0));
        return (x) => x * m;
    }
    // gentle rolling bumps
    const amp = p.amp ?? 16, wl = p.wavelength ?? 240;
    return (x) => amp * Math.sin((2 * Math.PI * x) / wl);
}

const smooth = (s) => s * s * (3 - 2 * s);

// Alternating walking stride for a given body progress `distance` (mm).
// geom: { flOff, rlOff } leg-centre offsets. opts: { terrain, stepLen, swingFrac }.
//
// Returns, per frame:
//   feet:     each leg's ON-GROUND foot position used by the SOLVE. Both stay on
//             the terrain and advance forward in alternating steps, so the
//             equilibrium is always the stable 2-contact case (no body sway).
//   swingSide / s: which leg is mid-step and how far through its swing (0..1).
//
// The swing leg's visible stepping motion is produced elsewhere by sweeping that
// leg's THIGH angles and letting the foot follow the linkage (forward kinematics)
// — see main.drawWalk. So the foot is seen to lift and step as a consequence of
// the thigh servos moving, which is what we want to visualise.
export function walkStep(distance, geom, opts = {}) {
    const terrain = opts.terrain || (() => 0);
    const stepLen = opts.stepLen ?? 60;
    const swingFrac = opts.swingFrac ?? 0.45;
    const centerX = distance;
    const shared = centerX / stepLen;                   // shared step clock

    // clockOff staggers the two legs (0 vs 0.5 -> alternate; never both swinging
    // since swingFrac < 0.5). hipAtStart compensates spatially so each foot still
    // plants near its own hip regardless of the timing offset.
    const oneFoot = (legOffset, clockOff) => {
        const c = shared + clockOff;
        const n = Math.floor(c), frac = c - n;
        const hipAtStart = (n - clockOff) * stepLen + legOffset;
        const plantX = hipAtStart + 0.5 * stepLen;
        let x = plantX, s = 0, swinging = false;
        if (frac >= 1 - swingFrac) {                    // swing: foot advances
            s = (frac - (1 - swingFrac)) / swingFrac;
            x = plantX + stepLen * smooth(s);
            swinging = true;
        }
        return { x, groundY: terrain(x), s, swinging };
    };

    const fl = oneFoot(geom.flOff, 0);
    const rl = oneFoot(geom.rlOff, 0.5);
    const swingSide = fl.swinging ? 'fl' : (rl.swinging ? 'rl' : null);
    const s = swingSide === 'fl' ? fl.s : (swingSide === 'rl' ? rl.s : 0);
    return {
        centerX,
        feet: { fl: { x: fl.x, groundY: fl.groundY }, rl: { x: rl.x, groundY: rl.groundY } },
        swingSide, s,
    };
}

// Thigh-angle trajectory for a swinging leg. Sweeping both thighs UP lifts the
// foot (FK), and front/rear biasing moves it back/forward — so this sweep traces
// a natural step arc: back & down -> up -> forward & down. Returns absolute angles
// around the leg's base (stance) thigh angles, clamped to the servo range.
export function swingThighs(baseFront, baseRear, s, opts = {}) {
    const fwd = opts.fwd ?? 32;     // fore/aft bias amplitude
    const lift = opts.lift ?? 38;   // symmetric up-sweep (foot lift) amplitude
    const clamp = (a) => Math.max(91, Math.min(179, a));
    return {
        front: clamp(baseFront + fwd * (1 - 2 * s) + lift * Math.sin(Math.PI * s)),
        rear:  clamp(baseRear + fwd * (2 * s - 1) + lift * Math.sin(Math.PI * s)),
    };
}
