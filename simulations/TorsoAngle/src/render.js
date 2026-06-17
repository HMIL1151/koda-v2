// render.js — draws a solved scene (a Robot.results() object) and fills the
// readout / legend / warning panels. Imports the view layer for primitives.

import * as V from './view.js';
import { compressionColor } from './helpers.js';

const warnEl = document.getElementById('warn');
const outputsEl = document.getElementById('outputs');
const legendEl = document.getElementById('legend');

export function showWarn(t) { warnEl.textContent = t; warnEl.classList.remove('hidden'); }
export function hideWarn() { warnEl.classList.add('hidden'); }

export function showError(msg) { V.clear(); V.centeredText('Unreachable geometry: ' + msg, '#e74c3c'); hideWarn(); }

// Draw a terrain profile (function worldX -> worldY) filled down to the canvas
// bottom, across the visible x range.
function drawTerrain(heightAt) {
    const { ctx, canvas, VIEW } = V;
    const leftX = VIEW.camX - (VIEW.ox) / VIEW.scale;
    const rightX = VIEW.camX + (canvas.width - VIEW.ox) / VIEW.scale;
    ctx.save();
    ctx.beginPath();
    let first = true;
    for (let wx = leftX; wx <= rightX; wx += 4) {
        const p = V.cv({ x: wx, y: heightAt(wx) });
        if (first) { ctx.moveTo(p.x, p.y); first = false; } else ctx.lineTo(p.x, p.y);
    }
    const br = V.cv({ x: rightX, y: heightAt(rightX) });
    ctx.lineTo(br.x, canvas.height);
    ctx.lineTo(V.cv({ x: leftX, y: 0 }).x, canvas.height);
    ctx.closePath();
    ctx.fillStyle = '#10231a';
    ctx.fill();
    ctx.strokeStyle = '#27ae60'; ctx.lineWidth = 2.5; ctx.setLineDash([]);
    ctx.beginPath(); first = true;
    for (let wx = leftX; wx <= rightX; wx += 4) {
        const p = V.cv({ x: wx, y: heightAt(wx) });
        if (first) { ctx.moveTo(p.x, p.y); first = false; } else ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();
    ctx.restore();
}

// res: a Robot.results() object. opts: { showWeight, weight, terrain, drawFeet,
// swingSide }. drawFeet (optional) gives lifted foot positions for the walk
// animation; the solve always uses the on-ground positions in res.points.feet.
export function renderScene(res, opts = {}) {
    V.clear();
    const P = res.points;
    const dim = res.stable === false;
    V.ctx.save();
    if (dim) V.ctx.globalAlpha = 0.4;

    // Drawn feet may be lifted (walk swing); ground feet stay on the terrain.
    const drawFeetWorld = opts.drawFeet || P.feet;
    const feetW = { fl: V.cv(drawFeetWorld.fl), rl: V.cv(drawFeetWorld.rl) };
    const groundW = { fl: V.cv(P.feet.fl), rl: V.cv(P.feet.rl) };

    // Ground: terrain profile (walk) or the infinite plane line through the feet.
    if (opts.terrain) {
        drawTerrain(opts.terrain);
    } else {
        const dir = { x: P.feet.rl.x - P.feet.fl.x, y: P.feet.rl.y - P.feet.fl.y };
        const m = Math.hypot(dir.x, dir.y) || 1;
        const u = { x: dir.x / m, y: dir.y / m };
        const ext = 230;
        V.line(V.cv({ x: P.feet.fl.x - u.x * ext, y: P.feet.fl.y - u.y * ext }),
               V.cv({ x: P.feet.rl.x + u.x * ext, y: P.feet.rl.y + u.y * ext }), '#27ae60', 2.5, [9, 6]);
    }
    // Weight line: vertical drop from the CoM showing where it lands vs support.
    if (opts.showWeight) {
        const com = V.cv(P.com);
        const groundY = (groundW.fl.y + groundW.rl.y) / 2 + 60;
        V.line({ x: com.x, y: com.y }, { x: com.x, y: groundY }, res.stable ? '#2ecc71' : '#e74c3c', 1.5, [3, 4]);
    }

    // Knee positions may be overridden for a swing leg (drawn from swept thighs).
    const kneeW = (name) => V.cv((opts.drawKnees && opts.drawKnees[name]) || P.knees[name]);
    const swing = opts.swingSide || null;
    const swingLoad = opts.swingLoad ?? 1;     // 1 = planted/loaded, 0 = lifted/free

    // Calves coloured by compression gradient. The swing leg's effective load
    // ramps with swingLoad so its colour fades to "unloaded" (green) as it lifts
    // and deepens as it is driven back into the ground.
    const calf = (side, name, foot, comp, state) => {
        const a = kneeW(name), b = foot;
        const c = side === swing ? comp * swingLoad : comp;
        V.line(a, b, compressionColor(c, res.minComp, res.maxComp), 3);
        if (side !== swing && state !== 'ok') V.line(a, b, '#e6a23c', 1.5, [3, 4]);
    };
    calf('fl', 'flFront', feetW.fl, res.comps.flF, res.states.flF);
    calf('fl', 'flRear',  feetW.fl, res.comps.flR, res.states.flR);
    calf('rl', 'rlFront', feetW.rl, res.comps.rlF, res.states.rlF);
    calf('rl', 'rlRear',  feetW.rl, res.comps.rlR, res.states.rlR);

    // Thighs (hip -> knee, knee possibly overridden).
    V.line(V.cv(P.hips.flFront), kneeW('flFront'), '#4a6480', 4);
    V.line(V.cv(P.hips.flRear),  kneeW('flRear'),  '#4a6480', 4);
    V.line(V.cv(P.hips.rlFront), kneeW('rlFront'), '#4a6480', 4);
    V.line(V.cv(P.hips.rlRear),  kneeW('rlRear'),  '#4a6480', 4);

    // Torso.
    V.line(V.cv(P.torsoEnds.left), V.cv(P.torsoEnds.right), '#34495e', 8);

    for (const name of ['flFront', 'flRear', 'rlFront', 'rlRear']) V.dot(kneeW(name), 4, '#e67e22');
    for (const hp of Object.values(P.hips)) { V.dot(V.cv(hp), 5, '#1e2a36'); V.dot(V.cv(hp), 3, '#95a5a6'); }
    V.dot(feetW.fl, 5.5, '#e74c3c');
    V.dot(feetW.rl, 5.5, '#e74c3c');

    // Ground reaction arrows; the swing foot's reaction fades with its load so it
    // grows smoothly as the foot is driven into the ground.
    for (const key of ['fl', 'rl']) {
        const sc = key === swing ? swingLoad : 1;
        if (sc < 0.02) continue;
        const Rk = res.reactions[key], foot = feetW[key];
        V.arrow(foot, { x: foot.x + Rk.x * V.PX_PER_N * sc, y: foot.y + Rk.y * V.PX_PER_N * sc }, '#2ecc71', 2.5);
    }

    // CoM + weight vector.
    if (opts.showWeight) {
        const com = V.cv(P.com);
        V.dot(com, 5, '#f1c40f');
        const wTip = { x: com.x, y: com.y + opts.weight * V.PX_PER_N };
        V.arrow(com, wTip, '#f1c40f', 2.5);
        V.label({ x: com.x + 10, y: (com.y + wTip.y) / 2 }, 'W', '#f1c40f', 'left');
    }

    V.label({ x: V.cv(P.hips.flFront).x, y: V.cv(P.torsoEnds.left).y - 26 }, 'FRONT', '#5d6d7e');
    V.label({ x: V.cv(P.hips.rlRear).x,  y: V.cv(P.torsoEnds.right).y - 26 }, 'REAR', '#5d6d7e');
    V.ctx.restore();

    // Warnings.
    if (opts.showWeight && res.stable === false) {
        showWarn('Robot TIPS — the weight line falls past the ' + res.tipSide + ' foot. Widen the stance, lower the CoM, or reduce the angle.');
    } else if (res.atStop) {
        showWarn('A spring is against an end stop — its force reading is saturated (inaccurate) there, though the stop still supports the body.');
    } else {
        hideWarn();
    }
}

// ---------- readout panels ----------
function row(lbl, value, cls = '') {
    return `<div class="readout ${cls}"><span class="lbl">${lbl}</span><span class="val">${value}</span></div>`;
}
function compStr(c, s) {
    const tag = s === 'ext-stop' ? ' (ext-stop)' : s === 'comp-stop' ? ' (comp-stop)' : '';
    return c.toFixed(1) + tag;
}

export function showSimulateOutputs(res, extra = '') {
    const c = res.comps, st = res.states;
    const fmt = (v) => (v >= 0 ? '+' : '') + v.toFixed(1);
    const stanceTxt = res.stable
        ? 'standing (' + Math.round(res.tipMargin * 100) + '% margin)'
        : 'TIPPING / ' + res.tipSide + ' foot';
    outputsEl.innerHTML = '<h2>Outputs</h2>'
        + (extra ? row('Mode', extra) : '')
        + row('Ground angle', res.groundAngleDeg.toFixed(1) + '&deg;')
        + row('Torso angle', res.torsoAngleDeg.toFixed(2) + '&deg;')
        + row('Torso vs ground', fmt(res.deviationDeg) + '&deg;', 'small')
        + row('Stance', stanceTxt, res.stable ? 'accent' : 'bad')
        + row('Load front / rear', res.loadFL.toFixed(1) + ' / ' + res.loadRL.toFixed(1) + ' N')
        + row('Front springs', compStr(c.flF, st.flF) + ', ' + compStr(c.flR, st.flR) + ' mm', res.atStop ? 'warnrow' : 'small')
        + row('Rear springs', compStr(c.rlF, st.rlF) + ', ' + compStr(c.rlR, st.rlR) + ' mm', res.atStop ? 'warnrow' : 'small');

    legendEl.innerHTML = '<h2>Legend</h2>'
        + '<div><span class="sw dot red"></span> Foot (pinned, high friction)</div>'
        + '<div><span class="sw dot orange"></span> Knee</div>'
        + '<div><span class="sw arrow green"></span> Ground reaction</div>'
        + '<div><span class="sw dot yellow"></span> CoM &amp; weight line</div>'
        + '<div><span class="sw gradbar"></span> Calf compression: low &rarr; high</div>'
        + '<div><span class="sw line amberline"></span> Calf at an end stop (force saturated)</div>';
}

export function showInferOutputs(res) {
    const la = res.legAngles, conf = res.confidence;
    const confCls = conf >= 60 ? 'accent' : conf >= 25 ? 'warnrow' : 'bad';
    outputsEl.innerHTML = '<h2>Outputs</h2>'
        + row('Inferred ground', res.groundAngleDeg.toFixed(2) + '&deg;', res.confidence > 0 ? 'accent' : 'bad')
        + row('Confidence', (conf === null ? '—' : conf + '%'), confCls)
        + row('Load front / rear', res.loadFL.toFixed(1) + ' / ' + res.loadRL.toFixed(1) + ' N', 'small')
        + row('Front leg reaction', la.fl.toFixed(1) + '&deg;', 'small')
        + row('Rear leg reaction', la.rl.toFixed(1) + '&deg;', 'small');

    legendEl.innerHTML = '<h2>How this works</h2>'
        + `<p class="note">${res.confidenceNote || ''}</p>`
        + '<p class="note">Ground = line through both feet; orientation from assuming the '
        + '<b>net</b> reaction is vertical (= gravity at equilibrium). Confidence = how '
        + 'centred the load is between the feet — a foot with no load means the angle is '
        + 'not physically realisable.</p>';
}
