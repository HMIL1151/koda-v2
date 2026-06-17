// render.js — draws a solved optimiser frame: the foot's reachable workspace
// cloud, the winning stride x clearance rectangle, the leg linkage drawn reaching
// into that box, the evolutionary population as faint ghost boxes, a convergence
// sparkline, and the side-panel readouts (best score / stride / clearance / the
// five length values + algorithm status).

import * as view from './view.js';
import { footFK } from './kinematics.js';
import { DIMS } from './controls.js';

const COL = {
    cloud:  'rgba(52, 152, 219, 0.35)',
    ghost:  'rgba(230, 162, 60, 0.30)',
    box:    '#2ecc71',
    boxFill:'rgba(46, 204, 113, 0.12)',
    torso:  '#8fa5b5',
    thigh:  '#3498db',
    calf:   '#e67e22',
    joint:  '#ecf0f1',
    hip:    '#f1c40f',
    foot:   '#e74c3c',
    dim:    '#aebfc9',
    faint:  '#4a5d6e',
};

function sceneBounds(result, lengths) {
    const b = result.bounds, hx = lengths.hipSpacing / 2;
    return {
        minX: Math.min(b.minX, -hx) - 6,
        maxX: Math.max(b.maxX, hx) + 6,
        minY: Math.min(b.minY, 0) - 6,
        maxY: b.maxY + 6,
    };
}

// state: { result, lengths, ghosts, history, algoName, status, problem, playing }
export function renderScene(state) {
    const { result, lengths } = state;
    view.clear();
    view.fit(sceneBounds(result, lengths));

    // Foot workspace cloud.
    for (const p of result.points) view.pixel(p, 2.4, COL.cloud);

    // Evolutionary ghosts: each candidate's stride x clearance box, faint.
    for (const g of state.ghosts || []) {
        if (g.result && g.result.rect) view.rect(g.result.rect, { stroke: COL.ghost, width: 1 });
    }

    // Winning rectangle + dims.
    if (result.rect) {
        const r = result.rect;
        view.rect(r, { fill: COL.boxFill, stroke: COL.box, width: 2 });
        view.label({ x: (r.minX + r.maxX) / 2, y: r.minY }, `stride ${result.stride.toFixed(0)} mm`,
            COL.dim, { baseline: 'bottom', dy: -3 });
        view.label({ x: r.minX, y: (r.minY + r.maxY) / 2 }, `clear ${result.clearance.toFixed(0)}`,
            COL.dim, { align: 'right', dx: -4, font: 'bold 10px monospace' });
    }

    // Linkage reaching into the box (representative pose nearest the rect centre).
    if (result.centrePose) {
        const fk = footFK(lengths, result.centrePose.aFront, result.centrePose.aRear);
        if (fk) drawLinkage(fk);
    }

    drawSparkline(state.history);
    drawTitle(state);
    fillReadouts(state);
}

function drawLinkage(fk) {
    view.line(fk.hipF, fk.hipR, COL.torso, 4);
    view.line(fk.hipF, fk.kneeF, COL.thigh, 3);
    view.line(fk.hipR, fk.kneeR, COL.thigh, 3);
    view.line(fk.kneeF, fk.foot, COL.calf, 3);
    view.line(fk.kneeR, fk.foot, COL.calf, 3);
    view.dot(fk.hipF, 4, COL.hip); view.dot(fk.hipR, 4, COL.hip);
    view.dot(fk.kneeF, 3.5, COL.joint); view.dot(fk.kneeR, 3.5, COL.joint);
    view.dot(fk.foot, 4.5, COL.foot);
}

function drawTitle(state) {
    const s = state.status;
    const txt = `${state.algoName}${s ? `   ·   ${s.label}: ${s.value}` : ''}${state.playing ? '' : '   (paused)'}`;
    view.screenText(12, 10, txt, '#8fa5b5', { font: 'bold 12px monospace' });
}

// Small convergence inset (best score vs tick), bottom-left of the canvas.
function drawSparkline(history) {
    if (!history || history.length < 2) return;
    const ctx = view.ctx;
    const x0 = 12, y1 = view.canvas.height - 12, w = 150, h = 46, y0 = y1 - h;
    let lo = Infinity, hi = -Infinity;
    for (const v of history) { if (v < lo) lo = v; if (v > hi) hi = v; }
    const span = Math.max(1e-6, hi - lo);
    ctx.save();
    ctx.strokeStyle = COL.faint; ctx.lineWidth = 1;
    ctx.strokeRect(x0, y0, w, h);
    ctx.fillStyle = '#7f8c8d'; ctx.font = '9px monospace'; ctx.textBaseline = 'bottom';
    ctx.fillText('best score', x0 + 3, y0 - 1);
    ctx.beginPath();
    ctx.strokeStyle = COL.box; ctx.lineWidth = 1.5;
    const n = history.length;
    for (let i = 0; i < n; i++) {
        const x = x0 + (w * i) / (n - 1);
        const y = y1 - (h * (history[i] - lo)) / span;
        i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.restore();
}

function row(label, value, cls = '') {
    return `<div class="readout ${cls}"><span class="lbl">${label}</span><span class="val">${value}</span></div>`;
}

function fillReadouts(state) {
    const { result, lengths, problem } = state;
    const out = document.getElementById('outputs');
    const locked = (key) => !problem.names.includes(key);
    let html = '';
    html += row('Best score', `${result.score.toFixed(0)} mm²`, 'accent');
    html += row('Stride × clearance',
        `${result.stride.toFixed(1)} × ${result.clearance.toFixed(1)} mm`);
    if (state.status) html += row(state.status.label, state.status.value, 'small');
    html += `<h2 style="margin-top:10px">Best dimensions</h2>`;
    for (const d of DIMS) {
        const lock = locked(d.key) ? ' 🔒' : '';
        html += row(d.label, `${lengths[d.key].toFixed(1)} mm${lock}`, locked(d.key) ? 'small' : '');
    }
    out.innerHTML = html;
}
