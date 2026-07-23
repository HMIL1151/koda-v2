// Central parameter schema. Each control is described once here and used to
// (a) build the live control panel and (b) validate/apply params in headless export.
//
// A control: { key, label, type, min?, max?, step?, options?, unit?, group }
// Groups map to the four control families the user asked for.

export const GROUPS = {
  time: 'Speed & Duration',
  rate: 'Print / Build Rate',
  camera: 'Camera & Framing',
  look: 'Colors & Labels',
  overlays: 'Title & Overlays',
  callouts: 'Callouts',
  materials: 'Materials Panel',
  ratings: 'Ratings Panel',
};

// Controls shared by every scene.
export const SHARED_CONTROLS = [
  // --- Speed & duration ---
  { key: 'duration', label: 'Build duration', type: 'range', min: 3, max: 20, step: 0.5, unit: 's', group: 'time' },
  { key: 'holdTime', label: 'Hold at end', type: 'range', min: 0, max: 5, step: 0.5, unit: 's', group: 'time' },
  { key: 'speed', label: 'Preview speed', type: 'range', min: 0.1, max: 4, step: 0.1, unit: '×', group: 'time' },
  { key: 'detailLayers', label: 'Detailed layers', type: 'range', min: 1, max: 6, step: 1, unit: '', group: 'time' },
  { key: 'detailFrac', label: 'Slow-phase portion', type: 'range', min: 0.1, max: 0.8, step: 0.05, unit: '', group: 'time' },

  // --- Print / build rate ---
  { key: 'partSize', label: 'Part size', type: 'range', min: 0.6, max: 1.6, step: 0.05, unit: '×', group: 'rate' },
  { key: 'partHeight', label: 'Part height', type: 'range', min: 1.0, max: 4.0, step: 0.1, unit: '', group: 'rate' },
  { key: 'layerHeight', label: 'Layer height', type: 'range', min: 0.06, max: 0.4, step: 0.01, unit: '', group: 'rate' },
  { key: 'actorSpeed', label: 'Tool sweep speed', type: 'range', min: 0.3, max: 3, step: 0.1, unit: '×', group: 'rate' },

  // --- Camera & framing ---
  { key: 'camMode', label: 'Camera mode', type: 'select', group: 'camera',
    options: [ ['static', 'Static isometric'], ['orbit', 'Slow orbit'], ['push', 'Slow push-in'] ] },
  { key: 'camAzimuth', label: 'Azimuth', type: 'range', min: -180, max: 180, step: 1, unit: '°', group: 'camera' },
  { key: 'camElevation', label: 'Elevation', type: 'range', min: 8, max: 70, step: 1, unit: '°', group: 'camera' },
  { key: 'camZoom', label: 'Zoom', type: 'range', min: 0.5, max: 2.2, step: 0.02, unit: '×', group: 'camera' },
  { key: 'orbitSpeed', label: 'Orbit speed', type: 'range', min: -60, max: 60, step: 1, unit: '°/s', group: 'camera' },
  { key: 'pushAmount', label: 'Push-in amount', type: 'range', min: 0, max: 0.6, step: 0.02, unit: '', group: 'camera' },

  // --- Colors & labels ---
  { key: 'bgColor', label: 'Background', type: 'color', group: 'look' },
  { key: 'partColor', label: 'Part color', type: 'color', group: 'look' },
  { key: 'accentColor', label: 'Process accent', type: 'color', group: 'look' },
  { key: 'showMachine', label: 'Show machine parts', type: 'toggle', group: 'look' },

  // --- Title & overlays ---
  { key: 'showTitle', label: 'Show title', type: 'toggle', group: 'overlays' },
  { key: 'titleText', label: 'Title text', type: 'text', group: 'overlays' },
  { key: 'titleScale', label: 'Title size', type: 'range', min: 0.5, max: 2.2, step: 0.05, unit: '×', group: 'overlays' },
  { key: 'titleX', label: 'Title X', type: 'range', min: -0.9, max: 0.9, step: 0.02, unit: '', group: 'overlays' },
  { key: 'titleY', label: 'Title Y', type: 'range', min: -0.05, max: 1.7, step: 0.02, unit: '', group: 'overlays' },
  { key: 'showFrame', label: 'Camera frame guide', type: 'toggle', group: 'overlays' },
  { key: 'calloutScale', label: 'Callout size', type: 'range', min: 0.5, max: 2, step: 0.05, unit: '×', group: 'overlays' },
  { key: 'showMaterials', label: 'Show materials panel', type: 'toggle', group: 'materials' },
  { key: 'showRatings', label: 'Show ratings panel', type: 'toggle', group: 'ratings' },
];

// Defaults shared across scenes.
export const SHARED_DEFAULTS = {
  duration: 8,
  holdTime: 2,
  speed: 1,
  detailLayers: 2,
  detailFrac: 0.45,

  showTitle: true,
  titleScale: 1,
  titleX: 0,
  titleY: 0,
  showFrame: false,
  calloutScale: 1,
  showMaterials: true,
  showRatings: true,

  partSize: 1,
  partHeight: 2.2,
  layerHeight: 0.14,
  actorSpeed: 1,

  camMode: 'static',
  camAzimuth: 35,
  camElevation: 28,
  camZoom: 1,
  orbitSpeed: 12,
  pushAmount: 0.28,

  bgColor: '#10141b',
  partColor: '#d9dde3',
  accentColor: '#4c9aff',
  showMachine: true,
};

// Rating categories shared by all processes (0–5 stars; higher = better).
export const RATING_CATS = ['Resolution', 'Surface finish', 'Speed', 'Strength', 'Low cost', 'Material range'];

// Default on-screen label positions per callout index (normalized within the
// 16:9 frame: 0 = centre, ±1 = edges). Labels are screen-anchored (semi-static)
// so they don't jump during a slow orbit; only the leader line tracks the part.
export const CALLOUT_LAYOUT = [[-0.4, -0.34], [0.4, -0.34], [-0.4, 0.42], [0.4, 0.42], [0, -0.42], [0, 0.46]];

// Callout ids/text per scene; each scene file maps the id → a world anchor.
// Per-scene metadata + accent/color defaults + optional extra controls.
export const SCENES = {
  fdm: {
    name: 'FDM',
    full: 'Fused Deposition Modeling',
    accent: '#ff7a3c',
    part: '#e7523b',
    materials: ['PLA', 'PETG', 'ABS / ASA', 'TPU (flexible)', 'Nylon', 'Polycarbonate'],
    ratings: [2.5, 2, 4, 3, 5, 5],
    callouts: [
      { id: 'nozzle', text: 'Hot-end / nozzle' },
      { id: 'part', text: 'Part' },
      { id: 'plate', text: 'Heated bed' },
      { id: 'infill', text: 'Infill' },
    ],
    extra: [
      { key: 'nozzleTemp', label: 'Melt glow', type: 'range', min: 0, max: 1, step: 0.05, unit: '', group: 'look' },
    ],
    defaults: { accentColor: '#ff7a3c', partColor: '#e7523b' },
  },
  sla: {
    name: 'SLA',
    full: 'Stereolithography',
    accent: '#7c5cff',
    part: '#8fd4e8',
    materials: ['Standard resin', 'Tough / ABS-like', 'Flexible', 'Castable', 'Dental / bio', 'High-temp'],
    ratings: [5, 5, 3, 2.5, 3, 3],
    callouts: [
      { id: 'part', text: 'Part' },
      { id: 'resin', text: 'Liquid resin' },
      { id: 'plate', text: 'Build plate' },
      { id: 'uv', text: 'UV laser (from below)' },
    ],
    extra: [
      { key: 'resinLevel', label: 'Resin opacity', type: 'range', min: 0.1, max: 0.9, step: 0.05, unit: '', group: 'look' },
    ],
    defaults: { accentColor: '#7c5cff', partColor: '#8fd4e8', camElevation: 16 },
  },
  sls: {
    name: 'SLS',
    full: 'Selective Laser Sintering',
    accent: '#ff3b6b',
    part: '#c9cdd4',
    materials: ['PA12 Nylon', 'PA11 Nylon', 'PA-GF (glass-filled)', 'TPU', 'Alumide'],
    ratings: [4, 3.5, 3.5, 4.5, 2.5, 3.5],
    callouts: [
      { id: 'laser', text: 'Sintering laser' },
      { id: 'part', text: 'Part' },
      { id: 'powder', text: 'Powder bed' },
      { id: 'recoater', text: 'Recoater' },
    ],
    extra: [
      { key: 'powderDarkness', label: 'Powder tone', type: 'range', min: 0.2, max: 0.9, step: 0.05, unit: '', group: 'look' },
    ],
    defaults: { accentColor: '#ff3b6b', partColor: '#c9cdd4' },
  },
  mjf: {
    name: 'MJF',
    full: 'Multi Jet Fusion',
    accent: '#25c2a0',
    part: '#3b3f46',
    materials: ['PA12 Nylon', 'PA11 Nylon', 'PA12-GB (glass-bead)', 'TPU', 'Polypropylene'],
    ratings: [4, 4, 4.5, 4.5, 3, 3],
    callouts: [
      { id: 'lamp', text: 'IR fusing lamp' },
      { id: 'carriage', text: 'Agent carriage' },
      { id: 'part', text: 'Part' },
      { id: 'powder', text: 'Powder bed' },
    ],
    extra: [
      { key: 'agentDarkness', label: 'Fusing-agent tone', type: 'range', min: 0.2, max: 0.95, step: 0.05, unit: '', group: 'look' },
    ],
    defaults: { accentColor: '#25c2a0', partColor: '#3b3f46' },
  },
};

// Dynamic per-scene controls: a toggle per material and a 0–5 slider per rating
// category. (Callouts get a dedicated editor in the UI, keyed on the same state.)
export function dynamicControls(sceneKey) {
  const s = SCENES[sceneKey];
  const out = [];
  (s.materials || []).forEach((m, i) => {
    out.push({ key: `mat_${i}`, label: m, type: 'toggle', group: 'materials' });
  });
  RATING_CATS.forEach((cat, i) => {
    out.push({ key: `rt_${i}`, label: cat, type: 'range', min: 0, max: 5, step: 0.5, unit: '★', group: 'ratings' });
  });
  return out;
}

export function defaultsForScene(sceneKey) {
  const s = SCENES[sceneKey];
  const extraDefaults = {};
  const EX = { nozzleTemp: 0.8, resinLevel: 0.5, powderDarkness: 0.55, agentDarkness: 0.85 };
  for (const c of (s.extra || [])) extraDefaults[c.key] = EX[c.key];
  const dyn = { titleText: s.full };
  (s.callouts || []).forEach((c, i) => {
    const [lx, ly] = CALLOUT_LAYOUT[i] || [0, 0];
    dyn[`co_${c.id}`] = true;
    dyn[`co_${c.id}_text`] = c.text;
    dyn[`co_${c.id}_lx`] = lx; dyn[`co_${c.id}_ly`] = ly;
    dyn[`co_${c.id}_mode`] = 'part';
    dyn[`co_${c.id}_tx`] = 0; dyn[`co_${c.id}_ty`] = 0; dyn[`co_${c.id}_tz`] = 0;
    dyn[`co_${c.id}_sx`] = lx * 0.5; dyn[`co_${c.id}_sy`] = ly * 0.5;
  });
  (s.materials || []).forEach((m, i) => { dyn[`mat_${i}`] = true; });
  (s.ratings || []).forEach((v, i) => { dyn[`rt_${i}`] = v; });
  return { ...SHARED_DEFAULTS, ...extraDefaults, ...dyn, ...(s.defaults || {}) };
}

// Controls for the callout editor (one callout at a time), keyed by id.
// Target mode: 'part' pins the leader to an XYZ on the part (tracks it as the
// camera orbits); 'screen' pins it to a fixed spot in the shot (stays put).
export function calloutControls(id, mode) {
  const rows = [
    { key: `co_${id}`, label: 'Visible', type: 'toggle', group: 'callouts' },
    { key: `co_${id}_text`, label: 'Text', type: 'text', group: 'callouts' },
    { key: `co_${id}_lx`, label: 'Label X', type: 'range', min: -1, max: 1, step: 0.02, group: 'callouts' },
    { key: `co_${id}_ly`, label: 'Label Y', type: 'range', min: -1, max: 1, step: 0.02, group: 'callouts' },
    { key: `co_${id}_mode`, label: 'Target', type: 'select', group: 'callouts',
      options: [['part', 'Point on part (3D)'], ['screen', 'Fixed in shot']] },
  ];
  if (mode === 'screen') {
    rows.push(
      { key: `co_${id}_sx`, label: 'Target X (shot)', type: 'range', min: -1, max: 1, step: 0.02, group: 'callouts' },
      { key: `co_${id}_sy`, label: 'Target Y (shot)', type: 'range', min: -1, max: 1, step: 0.02, group: 'callouts' },
    );
  } else {
    rows.push(
      { key: `co_${id}_tx`, label: 'Target X (part)', type: 'range', min: -5, max: 5, step: 0.1, group: 'callouts' },
      { key: `co_${id}_ty`, label: 'Target Y (part)', type: 'range', min: -3, max: 6, step: 0.1, group: 'callouts' },
      { key: `co_${id}_tz`, label: 'Target Z (part)', type: 'range', min: -5, max: 5, step: 0.1, group: 'callouts' },
    );
  }
  return rows;
}

export function controlsForScene(sceneKey) {
  const s = SCENES[sceneKey];
  return [...SHARED_CONTROLS, ...(s.extra || []), ...dynamicControls(sceneKey)];
}
