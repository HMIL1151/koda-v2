// controls.js — owns every DOM input. Builds one row per tunable dimension (value
// + min + max + Lock), the angle-range / grid-quality inputs, the algorithm picker
// and the transport bar, then bundles the current settings into a `problem` object
// for the optimisers. Locked dimensions are held at their value and excluded from
// the search vector, so "hold constant" works the same for every algorithm.

import { evaluate } from './objective.js';
import { OPTIMISER_NAMES } from './optimisers.js';
import { clamp } from './helpers.js';

// The five mechanism dimensions. `lo`/`hi` are the default search-bound values
// (themselves editable in the UI); `def` is the starting length.
export const DIMS = [
    { key: 'thighFront', label: 'Front thigh', def: 40,  lo: 10, hi: 120 },
    { key: 'thighRear',  label: 'Rear thigh',  def: 40,  lo: 10, hi: 120 },
    { key: 'calfFront',  label: 'Front calf',  def: 120, lo: 40, hi: 200 },
    { key: 'calfRear',   label: 'Rear calf',   def: 120, lo: 40, hi: 200 },
    { key: 'hipSpacing', label: 'Hip spacing', def: 46,  lo: 10, hi: 120 },
];

const QUALITY = { fast: 40, medium: 60, fine: 80 };   // angle-sweep grid N

const els = {};        // els[key] = { value, min, max, lock }
let ui = {};           // misc controls (algo, speed, angle range, quality)
const num = (el) => parseFloat(el.value);

function makeInput(cls, value, step = 1) {
    const i = document.createElement('input');
    i.type = 'number'; i.className = cls; i.value = value; i.step = step;
    return i;
}

function buildVarRows(onParamChange) {
    const host = document.getElementById('varRows');
    for (const d of DIMS) {
        const row = document.createElement('div');
        row.className = 'varrow';

        const head = document.createElement('div');
        head.className = 'varhead';
        const lock = document.createElement('input');
        lock.type = 'checkbox'; lock.className = 'lock';
        const lockLbl = document.createElement('label');
        lockLbl.className = 'locklbl'; lockLbl.title = 'Hold this dimension constant';
        lockLbl.append(lock, document.createTextNode(' lock'));
        const name = document.createElement('span');
        name.className = 'varname'; name.textContent = d.label;
        head.append(name, lockLbl);

        const grid = document.createElement('div');
        grid.className = 'vargrid';
        const value = makeInput('vin', d.def);
        const min   = makeInput('vin small', d.lo);
        const max   = makeInput('vin small', d.hi);
        grid.append(
            tagged('value', value), tagged('min', min), tagged('max', max));

        row.append(head, grid);
        host.append(row);
        els[d.key] = { value, min, max, lock };

        for (const el of [value, min, max]) el.addEventListener('input', onParamChange);
        lock.addEventListener('change', onParamChange);
    }
}

function tagged(label, input) {
    const wrap = document.createElement('label');
    wrap.className = 'vintag';
    const s = document.createElement('span'); s.textContent = label;
    wrap.append(s, input);
    return wrap;
}

function buildAlgoOptions() {
    const sel = document.getElementById('algo');
    for (const [val, label] of Object.entries(OPTIMISER_NAMES)) {
        const o = document.createElement('option');
        o.value = val; o.textContent = label;
        sel.append(o);
    }
}

export function initControls(cb) {
    buildVarRows(cb.onParamChange);
    buildAlgoOptions();

    ui = {
        algo:     document.getElementById('algo'),
        speed:    document.getElementById('speed'),
        angleMin: document.getElementById('angleMin'),
        angleMax: document.getElementById('angleMax'),
        quality:  document.getElementById('quality'),
        playBtn:  document.getElementById('playBtn'),
        stepBtn:  document.getElementById('stepBtn'),
        resetBtn: document.getElementById('resetBtn'),
        adoptBtn: document.getElementById('adoptBtn'),
    };

    ui.algo.addEventListener('change', cb.onAlgoChange);
    ui.quality.addEventListener('change', cb.onParamChange);
    for (const el of [ui.angleMin, ui.angleMax]) el.addEventListener('input', cb.onParamChange);
    ui.playBtn.addEventListener('click', cb.onPlayToggle);
    ui.stepBtn.addEventListener('click', cb.onStep);
    ui.resetBtn.addEventListener('click', cb.onReset);
    ui.adoptBtn.addEventListener('click', cb.onAdopt);
}

export function getAlgorithm() { return ui.algo.value; }
export function getSpeed()     { return parseInt(ui.speed.value, 10); }

export function setPlaying(on) { ui.playBtn.textContent = on ? '⏸ Pause' : '▶ Play'; }

function readEvalOpts() {
    const minA = num(ui.angleMin), maxA = num(ui.angleMax);
    return {
        minAngle: Math.min(minA, maxA),
        maxAngle: Math.max(minA, maxA),
        N: QUALITY[ui.quality.value] || 60,
        cellMm: 2,
    };
}

// Bundle the current UI state into the optimiser `problem`. Locked dims are baked
// into `base` (and the objective) and kept out of the search vector.
export function readProblem() {
    const base = {};
    const names = [], vec0 = [], lower = [], upper = [];
    for (const d of DIMS) {
        const e = els[d.key];
        const lo = num(e.min), hi = num(e.max);
        const val = clamp(num(e.value), Math.min(lo, hi), Math.max(lo, hi));
        base[d.key] = val;
        if (!e.lock.checked) {
            names.push(d.key); vec0.push(val);
            lower.push(Math.min(lo, hi)); upper.push(Math.max(lo, hi));
        }
    }
    const evalOpts = readEvalOpts();
    const lengths = (vec) => { const L = { ...base }; names.forEach((n, i) => (L[n] = vec[i])); return L; };
    return {
        names, vec0, lower, upper, base, evalOpts, lengths,
        evaluate: (vec) => evaluate(lengths(vec), evalOpts),
    };
}

// Write a lengths object back into the value inputs (the "Adopt best" button).
export function setStartValues(L) {
    for (const d of DIMS) if (L[d.key] != null) els[d.key].value.value = (Math.round(L[d.key] * 10) / 10);
}
