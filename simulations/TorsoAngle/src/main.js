// main.js — thin orchestrator. Wires the controls, physics (Robot), renderer and
// animation engine together. Per-frame work lives in the modules; this file just
// reads inputs, solves, and draws.
//
// Module map:
//   helpers.js   math / vectors / minimiser / colour utils
//   leg.js       single-leg kinematics (now 2 thigh angles per leg)
//   robot.js     two-leg static equilibrium + ground-angle inference
//   view.js      canvas, world->screen transform, drawing primitives
//   render.js    draws a solved scene + readout/legend/warning panels
//   controls.js  DOM inputs, 4-thigh link toggle, params, mode switch
//   gait.js      terrain profiles + quasi-static walking stride
//   animation.js transport (play/pause/step/scrub/speed/reset) + video export

import { Robot } from './robot.js';
import * as controls from './controls.js';
import * as render from './render.js';
import * as anim from './animation.js';
import * as gait from './gait.js';
import { setCamera } from './view.js';

// Static (non-animating) compute+draw for the current mode.
function compute() {
    try {
        if (controls.getMode() === 'simulate') {
            const { robotOpts, groundAngleDeg, weightN } = controls.readSimParams();
            const robot = new Robot(robotOpts);
            const res = robot.solveEquilibrium(groundAngleDeg);
            setCamera(0);
            render.renderScene(res, { showWeight: true, weight: weightN });
            render.showSimulateOutputs(res);
        } else {
            const { robotOpts, comps } = controls.readInferParams();
            const robot = new Robot(robotOpts);
            const res = robot.inferFromCompressions(comps);
            setCamera(0);
            render.renderScene(res, { showWeight: false });
            render.showInferOutputs(res);
        }
    } catch (e) {
        render.showError(e.message);
    }
}

// --- animation callbacks ---

// Sweep scenario: set the ground angle and run the normal static solve.
function drawSweep(groundDeg) {
    controls.setGroundAngle(groundDeg);
    compute();
}

// Walk scenario: march the body across terrain. The SOLVE keeps both feet on the
// ground (stable double support, no body sway). The swinging leg is drawn from its
// swept THIGH angles via forward kinematics so its foot lifts and steps via the
// linkage. Its whole configuration (foot, knees, spring load) is BLENDED toward
// the planted/loaded stance pose at both ends of the swing — so the spring
// compression ramps smoothly up as the foot is driven into the ground (and back
// down at lift-off) with no positional or load jump at the hand-off.
const lerp = (a, b, t) => ({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t });
const smoothstep = (a, b, x) => { const t = Math.max(0, Math.min(1, (x - a) / (b - a))); return t * t * (3 - 2 * t); };

function drawWalk(distance, terrain) {
    try {
        const { robotOpts, weightN } = controls.readSimParams();
        const robot = new Robot(robotOpts);
        const { centerX, feet, swingSide, s } = gait.walkStep(
            distance, { flOff: robot.flOff, rlOff: robot.rlOff }, { terrain });

        const res = robot.solveWithFeet({ x: feet.fl.x, y: feet.fl.groundY },
                                        { x: feet.rl.x, y: feet.rl.groundY });
        setCamera(centerX);

        let drawFeet, drawKnees, swingLoad = 1;
        if (swingSide) {
            // Lift envelope: exactly 0 near the start/end of the swing (so the leg
            // matches the planted/loaded stance pose at the hand-offs — no jump),
            // ~1 through the middle (fully lifted, unloaded).
            const L = smoothstep(0.08, 0.30, s) * (1 - smoothstep(0.70, 0.92, s));
            swingLoad = 1 - L;                          // load fades out then back in

            const baseFront = swingSide === 'fl' ? robotOpts.flFrontDeg : robotOpts.rlFrontDeg;
            const baseRear  = swingSide === 'fl' ? robotOpts.flRearDeg  : robotOpts.rlRearDeg;
            const sw = gait.swingThighs(baseFront, baseRear, s);
            const fk = robot.legWorldFK(swingSide, sw.front, sw.rear);

            const f = swingSide + 'Front', r = swingSide + 'Rear';
            drawFeet = { ...res.points.feet, [swingSide]: lerp(res.points.feet[swingSide], fk.foot, L) };
            drawKnees = {
                [f]: lerp(res.points.knees[f], fk.kneeF, L),
                [r]: lerp(res.points.knees[r], fk.kneeR, L),
            };
        }
        render.renderScene(res, { showWeight: true, weight: weightN, terrain, drawFeet, drawKnees, swingSide, swingLoad });
        render.showSimulateOutputs(res, 'walking');
    } catch (e) {
        render.showError(e.message);
    }
}

const { setMode } = controls.initControls({
    onChange: compute,
    onMode: (m) => { if (m !== 'simulate') anim.pause(); compute(); },
});
anim.initAnimation({ drawSweep, drawWalk });

setMode('simulate');
compute();
