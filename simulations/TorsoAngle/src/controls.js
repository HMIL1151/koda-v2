// controls.js — owns the DOM inputs: label sync, parameter reading, the 4-thigh
// link toggle, mode switching, and listener wiring.

const $ = (id) => document.getElementById(id);
const val = (id) => parseFloat($(id).value);

// id -> how to render its value label (suffix + decimals).
const LABELS = {
    thigh_flF: ['°', 0], thigh_flR: ['°', 0], thigh_rlF: ['°', 0], thigh_rlR: ['°', 0],
    legBase: [' mm', 0], groundAngle: ['°', 0], weight: [' N', 0],
    comOffset: [' mm', 0], comHeight: [' mm', 0], springRate: [' N/mm', 1],
    minComp: [' mm', 1], maxComp: [' mm', 0],
    flF: [' mm', 1], flR: [' mm', 1], rlF: [' mm', 1], rlR: [' mm', 1],
};
const THIGH_IDS = ['thigh_flF', 'thigh_flR', 'thigh_rlF', 'thigh_rlR'];

let mode = 'simulate';

export function getMode() { return mode; }

export function syncLabels() {
    for (const id of Object.keys(LABELS)) {
        const el = $(id + 'Val');
        if (!el) continue;
        const [suffix, dp] = LABELS[id];
        el.textContent = val(id).toFixed(dp) + suffix;
    }
}

export function thighAngles() {
    return { flF: val('thigh_flF'), flR: val('thigh_flR'),
             rlF: val('thigh_rlF'), rlR: val('thigh_rlR') };
}

// Robot constructor options shared by both modes.
function robotOpts() {
    const t = thighAngles();
    return {
        flFrontDeg: t.flF, flRearDeg: t.flR, rlFrontDeg: t.rlF, rlRearDeg: t.rlR,
        legBaseMm: val('legBase'),
        weightN: val('weight'),
        comOffsetMm: val('comOffset'),
        comHeightMm: val('comHeight'),
        springRateNmm: val('springRate'),
        minCompressionMm: val('minComp'),
        maxCompressionMm: val('maxComp'),
    };
}

export function readSimParams() {
    return { robotOpts: robotOpts(), groundAngleDeg: val('groundAngle'), weightN: val('weight') };
}

export function readInferParams() {
    return {
        robotOpts: robotOpts(),
        comps: { flF: val('flF'), flR: val('flR'), rlF: val('rlF'), rlR: val('rlR') },
    };
}

// Programmatic setters (used by the animation engine).
export function setGroundAngle(deg) {
    const el = $('groundAngle');
    el.value = Math.max(parseFloat(el.min), Math.min(parseFloat(el.max), deg));
    syncLabels();
}
export function setThighAngles(t) {
    for (const k of ['flF', 'flR', 'rlF', 'rlR']) {
        if (t[k] === undefined) continue;
        const el = $('thigh_' + k);
        el.value = Math.max(parseFloat(el.min), Math.min(parseFloat(el.max), t[k]));
    }
    syncLabels();
}

// Snapshot/restore for the animation Reset button.
const ALL = [...THIGH_IDS, 'legBase', 'groundAngle', 'weight', 'comOffset', 'comHeight',
             'springRate', 'minComp', 'maxComp', 'flF', 'flR', 'rlF', 'rlR'];
let defaults = null;
function captureDefaults() { defaults = {}; for (const id of ALL) defaults[id] = $(id).value; }
export function restoreDefaults() {
    if (!defaults) return;
    for (const id of ALL) $(id).value = defaults[id];
    syncLabels();
}

// When "link pairs" is on, each leg's two thighs move together (front-leg pair
// flF=flR, rear-leg pair rlF=rlR) — i.e. the original two-angle behaviour.
function applyLink(changedId) {
    if (!$('linkThighs').checked) return;
    if (changedId === 'thigh_flF') $('thigh_flR').value = $('thigh_flF').value;
    else if (changedId === 'thigh_flR') $('thigh_flF').value = $('thigh_flR').value;
    else if (changedId === 'thigh_rlF') $('thigh_rlR').value = $('thigh_rlF').value;
    else if (changedId === 'thigh_rlR') $('thigh_rlF').value = $('thigh_rlR').value;
}

export function initControls({ onChange, onMode }) {
    captureDefaults();
    syncLabels();

    for (const id of ALL) {
        $(id).addEventListener('input', () => { applyLink(id); syncLabels(); onChange(); });
    }
    $('linkThighs').addEventListener('change', () => {
        if ($('linkThighs').checked) {        // snap each pair together immediately
            $('thigh_flR').value = $('thigh_flF').value;
            $('thigh_rlR').value = $('thigh_rlF').value;
        }
        syncLabels(); onChange();
    });

    const setMode = (m) => {
        mode = m;
        $('modeSimulate').classList.toggle('active', m === 'simulate');
        $('modeInfer').classList.toggle('active', m === 'infer');
        $('simInputs').classList.toggle('hidden', m !== 'simulate');
        $('inferInputs').classList.toggle('hidden', m !== 'infer');
        $('animbar').classList.toggle('hidden', m !== 'simulate');
        onMode(m);
    };
    $('modeSimulate').addEventListener('click', () => setMode('simulate'));
    $('modeInfer').addEventListener('click', () => setMode('infer'));
    return { setMode };
}
