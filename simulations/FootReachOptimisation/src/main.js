// main.js — thin orchestrator. Reads the UI into a `problem`, builds the chosen
// optimiser, and runs a requestAnimationFrame loop that advances the search a few
// ticks per frame and redraws the current best. All the real work lives in the
// modules:
//   helpers.js     math / geometry / vectors / minimiser
//   kinematics.js  single-leg 5-bar forward kinematics + workspace sampling
//   objective.js   stride x clearance reach metric (largest inscribed rectangle)
//   optimisers.js  simulated annealing / evolutionary / hill-climb / gradient
//   controls.js    DOM inputs: per-dimension value/min/max/lock + transport
//   view.js        canvas, auto-fit world->screen transform, primitives
//   render.js      draws the cloud, box, linkage, ghosts, sparkline, readouts

import * as controls from './controls.js';
import * as render from './render.js';
import * as view from './view.js';
import { footIK } from './kinematics.js';
import { makeOptimiser, OPTIMISER_NAMES } from './optimisers.js';

let problem, optimiser, playing = false, rafId = null;
let manualFoot = null;          // user-dragged foot target (world mm), or null

function build() {
    manualFoot = null;          // a fresh search invalidates any held foot pose
    problem = controls.readProblem();
    optimiser = makeOptimiser(controls.getAlgorithm(), problem);
}

// Foot the dragged target maps to: the cursor if reachable, else the nearest point
// of the solved workspace cloud (so the leg tracks the reach boundary).
function solveManual(lengths, points) {
    if (!manualFoot) return null;
    let ik = footIK(lengths, manualFoot);
    if (!ik && points && points.length) ik = footIK(lengths, nearestPoint(points, manualFoot));
    return ik;
}

function nearestPoint(points, p) {
    let best = points[0], bestD = Infinity;
    for (const q of points) {
        const d = (q.x - p.x) ** 2 + (q.y - p.y) ** 2;
        if (d < bestD) { bestD = d; best = q; }
    }
    return best;
}

function draw() {
    const best = optimiser.best();
    const lengths = problem.lengths(best.vec);
    render.renderScene({
        result: best.result,
        lengths,
        manualLinkage: solveManual(lengths, best.result.points),
        ghosts: optimiser.ghosts(),
        history: optimiser.history,
        status: optimiser.status(),
        algoName: OPTIMISER_NAMES[controls.getAlgorithm()],
        problem,
        playing,
    });
}

function loop() {
    if (!playing) return;
    const steps = controls.getSpeed();
    for (let i = 0; i < steps; i++) optimiser.step();
    draw();
    rafId = requestAnimationFrame(loop);
}

function setPlaying(on) {
    playing = on;
    if (on) manualFoot = null;          // resuming the search drops the held foot pose
    controls.setPlaying(on);
    if (on && rafId === null) rafId = requestAnimationFrame(loop);
    if (!on && rafId !== null) { cancelAnimationFrame(rafId); rafId = null; }
}

// --- Click-drag the foot --------------------------------------------------------
// Pointer down/move sets the foot target; draw() solves the linkage by IK. Dragging
// pauses the search so the auto-fit view stays put while you pose the leg.
function pointerToWorld(ev) {
    const c = view.eventToCanvas(ev);
    return view.toWorld(c.x, c.y);
}

view.canvas.addEventListener('pointerdown', (ev) => {
    setPlaying(false);
    manualFoot = pointerToWorld(ev);
    view.canvas.setPointerCapture(ev.pointerId);
    view.canvas.style.cursor = 'grabbing';
    draw();
});
view.canvas.addEventListener('pointermove', (ev) => {
    if (!view.canvas.hasPointerCapture(ev.pointerId)) return;
    manualFoot = pointerToWorld(ev);
    draw();
});
const endDrag = (ev) => {
    if (view.canvas.hasPointerCapture(ev.pointerId)) view.canvas.releasePointerCapture(ev.pointerId);
    view.canvas.style.cursor = 'grab';
};
view.canvas.addEventListener('pointerup', endDrag);
view.canvas.addEventListener('pointercancel', endDrag);
view.canvas.style.cursor = 'grab';

controls.initControls({
    onPlayToggle: () => setPlaying(!playing),
    onStep:       () => { optimiser.step(); draw(); },
    onReset:      () => { setPlaying(false); build(); draw(); },
    onParamChange:() => { build(); draw(); },     // bounds/locks/quality changed -> fresh search
    onAlgoChange: () => { optimiser = makeOptimiser(controls.getAlgorithm(), problem); draw(); },
    onAdopt:      () => {
        controls.setStartValues(problem.lengths(optimiser.best().vec));
        setPlaying(false); build(); draw();
    },
});

build();
draw();
