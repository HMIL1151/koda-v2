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
import { makeOptimiser, OPTIMISER_NAMES } from './optimisers.js';

let problem, optimiser, playing = false, rafId = null;

function build() {
    problem = controls.readProblem();
    optimiser = makeOptimiser(controls.getAlgorithm(), problem);
}

function draw() {
    const best = optimiser.best();
    const lengths = problem.lengths(best.vec);
    render.renderScene({
        result: best.result,
        lengths,
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
    controls.setPlaying(on);
    if (on && rafId === null) rafId = requestAnimationFrame(loop);
    if (!on && rafId !== null) { cancelAnimationFrame(rafId); rafId = null; }
}

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
