// animation.js — transport engine over a deterministic timeline u in [0,1].
// Two scenarios: "sweep" (oscillate the ground angle) and "walk" (march across
// terrain). Because the timeline is a pure function of u, replay/scrub/step all
// work by re-evaluating u — no frame buffer needed. Also drives MediaRecorder to
// export the canvas to a video clip.

import { canvas } from './view.js';
import * as gait from './gait.js';
import * as controls from './controls.js';

const $ = (id) => document.getElementById(id);

const SWEEP_AMP = 12;        // deg, sweep scenario amplitude
const WALK_TOTAL = 1200;     // mm of body travel across the walk timeline
const STEP_FRAC = 1 / 180;   // timeline fraction per Step click

const A = {
    playing: false, raf: 0, u: 0, lastTs: 0,
    recorder: null, chunks: [], recording: false,
};

let cb = { drawSweep: () => {}, drawWalk: () => {} };

function durationMs() { return 6000 / Math.max(0.1, parseFloat($('animSpeed').value)); }
function animMode() { return $('animMode').value; }

function terrainFn() {
    const type = $('terrainSel').value;
    return gait.makeTerrain(type, { angleDeg: controls.readSimParams().groundAngleDeg });
}

// Render the frame at the current timeline position u.
function renderAt() {
    if (animMode() === 'walk') {
        cb.drawWalk(WALK_TOTAL * A.u, terrainFn());
    } else {
        cb.drawSweep(SWEEP_AMP * Math.sin(2 * Math.PI * A.u));
    }
    $('scrub').value = Math.round(A.u * 1000);
}

function frame(ts) {
    if (!A.playing) return;
    if (!A.lastTs) A.lastTs = ts;
    A.u += (ts - A.lastTs) / durationMs();
    A.lastTs = ts;
    if (A.u >= 1) {
        if (A.recording) { A.u = 1; renderAt(); stopRecording(); stop(); return; }
        A.u %= 1;                                  // loop
    }
    renderAt();
    A.raf = requestAnimationFrame(frame);
}

function play() {
    if (A.playing) return;
    A.playing = true; A.lastTs = 0;
    $('playBtn').innerHTML = '&#10073;&#10073; Pause';
    A.raf = requestAnimationFrame(frame);
}
function stop() {
    if (!A.playing) return;
    A.playing = false; cancelAnimationFrame(A.raf);
    $('playBtn').innerHTML = '&#9654; Play';
}
function step(dir) { stop(); A.u = Math.max(0, Math.min(1, A.u + dir * STEP_FRAC)); renderAt(); }
function reset() { stop(); A.u = 0; controls.restoreDefaults(); renderAt(); }

// ---- video export (MediaRecorder on the canvas stream) ----
function pickMime() {
    const cands = ['video/mp4;codecs=avc1', 'video/mp4',
                   'video/webm;codecs=vp9', 'video/webm;codecs=vp8', 'video/webm'];
    for (const m of cands) if (window.MediaRecorder && MediaRecorder.isTypeSupported(m)) return m;
    return '';
}
function setRecStatus(t) { $('recStatus').textContent = t; }
function startRecording() {
    if (!window.MediaRecorder || !canvas.captureStream) { setRecStatus('Recording unsupported.'); return; }
    const mime = pickMime();
    try {
        A.recorder = new MediaRecorder(canvas.captureStream(30), mime ? { mimeType: mime } : undefined);
    } catch (e) { setRecStatus('Recorder error: ' + e.message); return; }
    A.chunks = [];
    A.recorder.ondataavailable = (e) => { if (e.data.size) A.chunks.push(e.data); };
    A.recorder.onstop = () => {
        const type = A.recorder.mimeType || mime || 'video/webm';
        const ext = type.includes('mp4') ? 'mp4' : 'webm';
        const url = URL.createObjectURL(new Blob(A.chunks, { type }));
        const a = document.createElement('a');
        a.href = url; a.download = 'robot-' + animMode() + '.' + ext; a.click();
        URL.revokeObjectURL(url);
        setRecStatus('Saved .' + ext + (ext === 'webm' ? ' (no MP4 capture in this browser)' : ''));
        A.recording = false; $('recBtn').classList.remove('active');
    };
    A.recorder.start(); A.recording = true;
    $('recBtn').classList.add('active'); setRecStatus('Recording one loop…');
    A.u = 0; play();                               // record exactly one timeline pass
}
function stopRecording() { if (A.recorder && A.recorder.state !== 'inactive') A.recorder.stop(); }

export function initAnimation(callbacks) {
    cb = callbacks;
    $('playBtn').addEventListener('click', () => (A.playing ? stop() : play()));
    $('stepBack').addEventListener('click', () => step(-1));
    $('stepFwd').addEventListener('click', () => step(1));
    $('resetBtn').addEventListener('click', reset);
    $('scrub').addEventListener('input', () => { stop(); A.u = parseFloat($('scrub').value) / 1000; renderAt(); });
    $('animMode').addEventListener('change', renderAt);
    $('terrainSel').addEventListener('change', renderAt);
    $('recBtn').addEventListener('click', () => {
        if (A.recording) { stop(); stopRecording(); } else startRecording();
    });
}

// Stop playback when leaving simulate mode / params changed externally.
export function pause() { stop(); }
export function isPlaying() { return A.playing; }
