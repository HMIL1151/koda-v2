import * as THREE from 'three';
import { CameraRig } from './camera.js';
import { buildState } from './timeline.js';
import {
  GROUPS, SHARED_CONTROLS, SHARED_DEFAULTS, SCENES, RATING_CATS,
  defaultsForScene, controlsForScene, calloutControls,
} from './config.js';
import { HUD, fitRect } from './hud.js';
import { FDMScene } from './scenes/fdm.js';
import { SLAScene } from './scenes/sla.js';
import { SLSScene } from './scenes/sls.js';
import { MJFScene } from './scenes/mjf.js';

const EXPORT_ASPECT = 16 / 9;

const SCENE_CLASSES = { fdm: FDMScene, sla: SLAScene, sls: SLSScene, mjf: MJFScene };

const params = new URLSearchParams(location.search);
const HEADLESS = params.has('headless');

// ---------------------------------------------------------------- renderer
const canvas = document.getElementById('c');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true, alpha: false });
renderer.localClippingEnabled = true;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.12;
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.autoClear = false; // we manage clears for letterboxed framing + HUD compositing

const scene = new THREE.Scene();
const rig = new CameraRig(EXPORT_ASPECT);
const hud = new HUD();
const LETTERBOX = new THREE.Color(0x05070a);
let frameRect = { x: 0, y: 0, w: 16, h: 9 };

// ---------------------------------------------------------------- lighting
const hemi = new THREE.HemisphereLight(0xbcd3ff, 0x2a2620, 0.55);
scene.add(hemi);
const key = new THREE.DirectionalLight(0xffffff, 2.1);
key.position.set(6, 10, 7);
key.castShadow = true;
key.shadow.mapSize.set(2048, 2048);
key.shadow.camera.near = 1; key.shadow.camera.far = 40;
const sc = key.shadow.camera;
sc.left = -8; sc.right = 8; sc.top = 8; sc.bottom = -8;
key.shadow.bias = -0.0004;
scene.add(key);
const fill = new THREE.DirectionalLight(0x9fbaff, 0.5);
fill.position.set(-7, 4, -3);
scene.add(fill);
const rim = new THREE.DirectionalLight(0xffffff, 0.7);
rim.position.set(-2, 5, -9);
scene.add(rim);

// ground shadow catcher
const ground = new THREE.Mesh(
  new THREE.CircleGeometry(30, 64),
  new THREE.MeshStandardMaterial({ color: 0x0e131a, roughness: 1, metalness: 0 }),
);
ground.rotation.x = -Math.PI / 2;
ground.position.y = -0.26;
ground.receiveShadow = true;
scene.add(ground);

// ---------------------------------------------------------------- state
let sceneKey = 'fdm';
let PRESETS = {};                 // saved per-scene defaults (from presets.json / localStorage)
const sceneStates = {};           // live per-scene state, remembered across tab switches
let state = { ...defaultsForScene(sceneKey) };
let root = new THREE.Group();
let sceneObj = null;
scene.add(root);

// built-in defaults for a scene, overlaid with any saved preset
function sceneBase(key) {
  return { ...defaultsForScene(key), ...(PRESETS[key] || {}) };
}

function loadScene(key) {
  if (sceneObj) { sceneStates[sceneKey] = state; sceneObj.dispose(); scene.remove(root); }
  sceneKey = key;
  state = sceneStates[key] || sceneBase(key); // keep this scene's own tuning
  sceneStates[key] = state;
  root = new THREE.Group();
  scene.add(root);
  sceneObj = new SCENE_CLASSES[key]();
  sceneObj.build(root, state);
  applyBackground();
}

function applyBackground() {
  scene.background = new THREE.Color(state.bgColor);
  ground.material.color.set(state.bgColor).multiplyScalar(0.7);
}

// ---------------------------------------------------------------- render one frame
function renderAt(t) {
  applyBackground();
  // Hold: once the build finishes, the part freezes on its final state for
  // `holdTime`s, but the camera keeps moving (orbit/push continue through the hold).
  const te = Math.min(t, state.duration);
  rig.update(t, state);
  const bs = buildState(te, state);
  sceneObj.update(te, bs, state);

  const W = canvas.width, H = canvas.height;
  const fr = frameRect;

  // letterbox clear, then render the scene into the 16:9 frame
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, W, H);
  renderer.setClearColor(LETTERBOX, 1);
  renderer.clear(true, true, true);

  renderer.setViewport(fr.x, fr.y, fr.w, fr.h);
  renderer.setScissor(fr.x, fr.y, fr.w, fr.h);
  renderer.setScissorTest(true);
  renderer.setClearColor(new THREE.Color(state.bgColor), 1);
  renderer.clear(true, true, false);
  renderer.render(scene, rig.cam);

  // HUD overlay (title / callouts / panels / frame guide) over the full canvas
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, W, H);
  hud.setSize(W, H);
  hud.render(renderer, hudSpec(fr, bs));

  const total = state.duration + state.holdTime;
  if (badge) badge.textContent = `${SCENES[sceneKey].name}  ·  ${t.toFixed(2)}s / ${total.toFixed(1)}s${t > state.duration ? '  (hold)' : ''}`;
}

// Assemble the overlay spec from current params + the active scene.
function hudSpec(fr, bs) {
  const sc = SCENES[sceneKey];
  const title = { text: state.titleText ?? sc.full, scale: state.titleScale, x: state.titleX, y: state.titleY, show: state.showTitle };
  const callouts = (sceneObj.getCallouts ? sceneObj.getCallouts(bs, state) : [])
    .filter((c) => state[`co_${c.id}`])
    .map((c) => ({
      text: state[`co_${c.id}_text`] ?? c.id,
      world: c.world.clone().add(new THREE.Vector3(state[`co_${c.id}_tx`] || 0, state[`co_${c.id}_ty`] || 0, state[`co_${c.id}_tz`] || 0)),
      lx: state[`co_${c.id}_lx`] ?? 0,
      ly: state[`co_${c.id}_ly`] ?? 0,
      mode: state[`co_${c.id}_mode`] || 'part',
      sx: state[`co_${c.id}_sx`] ?? 0,
      sy: state[`co_${c.id}_sy`] ?? 0,
    }));
  const materials = {
    show: state.showMaterials, title: 'Common materials',
    items: sc.materials.filter((m, i) => state[`mat_${i}`]),
  };
  const ratings = {
    show: state.showRatings, title: 'Process rating',
    rows: RATING_CATS.map((cat, i) => ({ label: cat, stars: state[`rt_${i}`] ?? 0 })),
  };
  return {
    frameRect: fr, camera: rig.cam, accent: state.accentColor,
    title, callouts, calloutScale: state.calloutScale,
    materials, ratings,
    showFrame: state.showFrame && !HEADLESS, // framing guide is preview-only, never burned into exports
  };
}

// ---------------------------------------------------------------- sizing
function recomputeFrame() {
  // frameRect is the 16:9 region (in drawing-buffer px) the scene renders into.
  frameRect = fitRect(canvas.width, canvas.height, EXPORT_ASPECT);
  rig.setAspect(EXPORT_ASPECT);
}

function resizeLive() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.setSize(w, h, false);
  recomputeFrame();
}

function setExactSize(w, h) {
  renderer.setPixelRatio(1);
  renderer.setSize(w, h, false);
  canvas.style.width = w + 'px';
  canvas.style.height = h + 'px';
  recomputeFrame();
}

// ---------------------------------------------------------------- live loop
let playing = !HEADLESS;
let t = 0;
let last = performance.now();
const badge = document.getElementById('badge');

function tick(now) {
  const dt = Math.min(0.05, (now - last) / 1000);
  last = now;
  if (playing) {
    t += dt * state.speed;
    if (t >= state.duration + state.holdTime) t = 0;
    if (scrub) scrub.value = String(t);
  }
  renderAt(t);
  requestAnimationFrame(tick);
}

// ---------------------------------------------------------------- UI
let scrub = null;
function buildUI() {
  const tabs = document.getElementById('tabs');
  tabs.innerHTML = '';
  for (const k of Object.keys(SCENE_CLASSES)) {
    const el = document.createElement('div');
    el.className = 'tab' + (k === sceneKey ? ' active' : '');
    el.textContent = SCENES[k].name;
    el.onclick = () => { loadScene(k); t = 0; buildUI(); };
    tabs.appendChild(el);
  }

  const host = document.getElementById('controls');
  host.innerHTML = '';

  // transport
  const transport = groupEl('Playback');
  const row = document.createElement('div'); row.className = 'scrub';
  const playBtn = document.createElement('button'); playBtn.textContent = playing ? '⏸ Pause' : '▶ Play';
  playBtn.style.width = '92px';
  playBtn.onclick = () => { playing = !playing; playBtn.textContent = playing ? '⏸ Pause' : '▶ Play'; };
  scrub = document.createElement('input'); scrub.type = 'range'; scrub.min = 0; scrub.max = state.duration + state.holdTime; scrub.step = 0.01; scrub.value = t;
  scrub.style.flex = '1';
  scrub.oninput = () => { playing = false; playBtn.textContent = '▶ Play'; t = parseFloat(scrub.value); };
  row.append(playBtn, scrub);
  transport.appendChild(row);
  host.appendChild(transport);

  // save/reset defaults
  const def = groupEl('Defaults');
  const dhint = document.createElement('div'); dhint.className = 'hint';
  dhint.textContent = 'Save every scene’s current settings as the defaults used on load and in exports.';
  def.appendChild(dhint);
  const dbtns = document.createElement('div'); dbtns.className = 'btns';
  const saveBtn = document.createElement('button'); saveBtn.className = 'primary'; saveBtn.textContent = '💾 Save as default';
  saveBtn.onclick = () => saveDefaults(saveBtn);
  const resetBtn = document.createElement('button'); resetBtn.textContent = '↺ Reset';
  resetBtn.onclick = () => resetDefaults(resetBtn);
  dbtns.append(saveBtn, resetBtn);
  def.appendChild(dbtns);
  host.appendChild(def);

  // grouped controls
  const controls = controlsForScene(sceneKey);
  for (const gkey of Object.keys(GROUPS)) {
    const g = groupEl(GROUPS[gkey]);
    if (gkey === 'callouts') { buildCalloutEditor(g); host.appendChild(g); continue; }
    const inGroup = controls.filter((c) => c.group === gkey);
    for (const c of inGroup) g.appendChild(controlRow(c));
    host.appendChild(g);
  }

  // export
  const ex = groupEl('Export video');
  buildExportUI(ex);
  host.appendChild(ex);
  updateCmd();

  if (scrub) scrub.max = state.duration + state.holdTime;
}

// ---------------------------------------------------------------- presets (saved defaults)
const PRESET_KEY = 'am_presets_v1';

async function loadPresets() {
  try {
    const r = await fetch('presets.json', { cache: 'no-store' });
    if (r.ok) { PRESETS = await r.json(); return; }
  } catch { /* not served / not present */ }
  try { const ls = localStorage.getItem(PRESET_KEY); if (ls) PRESETS = JSON.parse(ls); } catch { /* ignore */ }
}

// snapshot all scenes (current one live, others from remembered/base state)
function gatherPresets() {
  sceneStates[sceneKey] = state;
  const out = {};
  for (const k of Object.keys(SCENE_CLASSES)) {
    const s = { ...(sceneStates[k] || sceneBase(k)) };
    delete s.speed; // preview-only
    out[k] = s;
  }
  return out;
}

async function saveDefaults(btn) {
  const presets = gatherPresets();
  PRESETS = presets;
  try { localStorage.setItem(PRESET_KEY, JSON.stringify(presets)); } catch { /* ignore */ }
  let label = '✓ Saved (browser only)';
  try {
    const r = await fetch('/api/presets', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(presets) });
    if (r.ok) label = '✓ Saved as default';
  } catch { /* server not available */ }
  btn.textContent = label;
  setTimeout(() => { btn.textContent = '💾 Save as default'; }, 1600);
}

async function resetDefaults(btn) {
  PRESETS = {};
  try { localStorage.removeItem(PRESET_KEY); } catch { /* ignore */ }
  try { await fetch('/api/presets', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ __reset: true }) }); } catch { /* ignore */ }
  for (const k of Object.keys(sceneStates)) delete sceneStates[k];
  const cur = sceneKey;
  loadScene(cur); t = 0; buildUI();
  btn.textContent = '✓ Reset';
  setTimeout(() => { btn.textContent = '↺ Reset'; }, 1600);
}

// export settings
const EXPORT = { res: '3840x2160', fps: 60, crf: 16 };
let exporting = false;

function buildExportUI(ex) {
  const mkSel = (label, opts, val, on) => {
    const row = document.createElement('div'); row.className = 'row';
    const l = document.createElement('label'); l.textContent = label;
    const sel = document.createElement('select');
    for (const [v, t] of opts) { const o = document.createElement('option'); o.value = v; o.textContent = t; if (v == val) o.selected = true; sel.appendChild(o); }
    sel.onchange = () => on(sel.value);
    row.append(l, sel); return row;
  };
  ex.appendChild(mkSel('Resolution', [['3840x2160', '4K · 2160p'], ['2560x1440', '1440p'], ['1920x1080', '1080p'], ['1280x720', '720p (fast)']], EXPORT.res, (v) => EXPORT.res = v));
  ex.appendChild(mkSel('Frame rate', [['60', '60 fps'], ['30', '30 fps']], String(EXPORT.fps), (v) => EXPORT.fps = +v));
  ex.appendChild(mkSel('Quality', [['16', 'High'], ['14', 'Very high'], ['20', 'Smaller file']], String(EXPORT.crf), (v) => EXPORT.crf = +v));

  const btns = document.createElement('div'); btns.className = 'btns';
  const go = document.createElement('button'); go.className = 'primary'; go.textContent = '🎬 Export video';
  const copy = document.createElement('button'); copy.textContent = 'Copy CLI cmd';
  copy.onclick = () => { navigator.clipboard?.writeText(document.getElementById('cmd').textContent); copy.textContent = 'Copied ✓'; setTimeout(() => copy.textContent = 'Copy CLI cmd', 1200); };
  btns.append(go, copy);
  ex.appendChild(btns);

  // progress bar
  const barWrap = document.createElement('div');
  barWrap.style.cssText = 'height:8px;border-radius:5px;background:#0b0f14;border:1px solid var(--line);margin-top:8px;overflow:hidden;display:none;';
  const bar = document.createElement('div');
  bar.style.cssText = 'height:100%;width:0%;background:var(--accent);transition:width .2s;';
  barWrap.appendChild(bar);
  ex.appendChild(barWrap);

  const status = document.createElement('div'); status.className = 'cmd'; status.id = 'exportStatus'; status.style.display = 'none';
  ex.appendChild(status);

  go.onclick = () => runExport(go, status, bar, barWrap);

  const hint = document.createElement('div'); hint.className = 'hint';
  hint.textContent = 'Renders on this machine (needs npm run serve) → out/<name>/ with PNG frames, a zip, and the mp4. Or copy the CLI command:';
  ex.appendChild(hint);
  const cmd = document.createElement('div'); cmd.className = 'cmd'; cmd.id = 'cmd';
  ex.appendChild(cmd);
}

async function runExport(btn, status, bar, barWrap) {
  if (exporting) return;
  const [w, h] = EXPORT.res.split('x').map(Number);
  const job = { scene: sceneKey, width: w, height: h, fps: EXPORT.fps, crf: EXPORT.crf, params: exportParams() };
  exporting = true; btn.disabled = true; btn.textContent = '⏳ Rendering…';
  status.style.display = 'block'; status.textContent = 'Starting…';
  barWrap.style.display = 'block'; bar.style.width = '0%';
  try {
    const resp = await fetch('/api/render', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(job) });
    if (!resp.ok) { status.textContent = 'Error: ' + (await resp.text()); return; }
    const reader = resp.body.getReader();
    const dec = new TextDecoder();
    let buf = '';
    for (;;) {
      const { done, value } = await reader.read();
      if (done) break;
      buf += dec.decode(value, { stream: true });
      const lines = buf.split(/[\r\n]+/).filter(Boolean);
      status.textContent = lines.slice(-5).join('\n');
      // progress: last "NN%" is the frame-render progress; then the ffmpeg encode
      const pcts = buf.match(/(\d+)%/g);
      let pct = pcts ? +pcts[pcts.length - 1].replace('%', '') : 0;
      if (/Encoding mp4/.test(buf)) { pct = 98; btn.textContent = '⏳ Encoding…'; }
      if (buf.includes('__DONE__')) pct = 100;
      bar.style.width = pct + '%';
    }
    if (buf.includes('__DONE__')) { status.textContent += '\n✅ Done — see the out/ folder.'; bar.style.width = '100%'; }
  } catch (e) {
    status.textContent = 'Export failed (is the page served via `npm run serve`?): ' + e.message;
  } finally {
    exporting = false; btn.disabled = false; btn.textContent = '🎬 Export video';
  }
}

function groupEl(title) {
  const g = document.createElement('div'); g.className = 'group';
  const t = document.createElement('div'); t.className = 'gt'; t.textContent = title;
  g.appendChild(t); return g;
}

let selectedCallout = 0;
function buildCalloutEditor(g) {
  const list = SCENES[sceneKey].callouts || [];
  if (!list.length) return;
  selectedCallout = Math.min(selectedCallout, list.length - 1);
  const hint = document.createElement('div'); hint.className = 'hint';
  hint.textContent = 'Labels are screen-anchored (stay put during orbit); the leader line tracks the part. Pick a callout to edit:';
  g.appendChild(hint);

  const sel = document.createElement('select');
  list.forEach((c, i) => { const o = document.createElement('option'); o.value = i; o.textContent = state[`co_${c.id}_text`] || c.text; if (i === selectedCallout) o.selected = true; sel.appendChild(o); });
  sel.onchange = () => { selectedCallout = +sel.value; buildUI(); };
  g.appendChild(sel);

  const id = list[selectedCallout].id;
  for (const c of calloutControls(id, state[`co_${id}_mode`])) g.appendChild(controlRow(c));
}

function controlRow(c) {
  const wrap = document.createElement('div'); wrap.className = 'row';
  const lab = document.createElement('label'); lab.textContent = c.label; wrap.appendChild(lab);

  if (c.type === 'range') {
    const val = document.createElement('span'); val.className = 'val';
    const fmt = () => `${(+state[c.key]).toFixed(c.step < 1 ? 2 : 0)}${c.unit || ''}`;
    val.textContent = fmt();
    const inp = document.createElement('input'); inp.type = 'range';
    inp.min = c.min; inp.max = c.max; inp.step = c.step; inp.value = state[c.key];
    inp.oninput = () => { state[c.key] = parseFloat(inp.value); val.textContent = fmt(); afterChange(c); };
    wrap.append(val, inp);
  } else if (c.type === 'select') {
    const sel = document.createElement('select');
    for (const [v, label] of c.options) {
      const o = document.createElement('option'); o.value = v; o.textContent = label;
      if (state[c.key] === v) o.selected = true; sel.appendChild(o);
    }
    sel.onchange = () => { state[c.key] = sel.value; afterChange(c); };
    wrap.appendChild(sel);
  } else if (c.type === 'color') {
    const inp = document.createElement('input'); inp.type = 'color'; inp.value = state[c.key];
    inp.oninput = () => { state[c.key] = inp.value; afterChange(c); };
    wrap.appendChild(inp);
  } else if (c.type === 'toggle') {
    const inp = document.createElement('input'); inp.type = 'checkbox'; inp.checked = !!state[c.key];
    inp.onchange = () => { state[c.key] = inp.checked; afterChange(c); };
    wrap.appendChild(inp);
  } else if (c.type === 'text') {
    const inp = document.createElement('input'); inp.type = 'text'; inp.value = state[c.key] ?? '';
    inp.style.gridColumn = '1 / -1';
    inp.oninput = () => { state[c.key] = inp.value; afterChange(c); };
    wrap.appendChild(inp);
  }
  return wrap;
}

function afterChange(c) {
  if ((c.key === 'duration' || c.key === 'holdTime') && scrub) scrub.max = state.duration + state.holdTime;
  updateCmd();
  if (c.key.endsWith('_mode')) buildUI(); // swap part/screen target controls
}

function exportParams() {
  // everything except preview-only playback speed and the framing guide
  const overrides = { ...state };
  delete overrides.speed;
  delete overrides.showFrame;
  return overrides;
}

function updateCmd() {
  const el = document.getElementById('cmd'); if (!el) return;
  const json = JSON.stringify(exportParams());
  el.textContent = `node render.mjs --scene ${sceneKey} --fps 60 --width 3840 --height 2160 \\\n  --params '${json}'`;
}

// ---------------------------------------------------------------- headless API
window.KODA = {
  ready: false,
  listScenes: () => Object.keys(SCENE_CLASSES),
  setScene: (k) => { loadScene(k); },
  setParams: (obj) => { Object.assign(state, obj || {}); },
  getParams: () => ({ ...state }),
  getDuration: () => state.duration + state.holdTime, // includes the end-hold
  setSize: (w, h) => setExactSize(w, h),
  // Deterministic: render exactly the frame at index for the given fps.
  renderFrame: (i, fps) => { renderAt(i / fps); },
  frameCount: (fps) => Math.round((state.duration + state.holdTime) * fps) + 1,
  grabPNG: () => canvas.toDataURL('image/png'),
};

// ---------------------------------------------------------------- boot
(async () => {
  await loadPresets(); // saved defaults apply to preview and headless exports alike
  loadScene(sceneKey);
  if (HEADLESS) {
    document.getElementById('panel')?.remove();
    document.getElementById('app').style.gridTemplateColumns = '1fr';
    setExactSize(3840, 2160);
  } else {
    buildUI();
    resizeLive();
    window.addEventListener('resize', resizeLive);
    requestAnimationFrame((n) => { last = n; requestAnimationFrame(tick); });
  }
  window.KODA.ready = true;
})();
