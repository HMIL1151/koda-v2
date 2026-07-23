import * as THREE from 'three';
import { mergeGeometries } from '../vendor/BufferGeometryUtils.js';

// ---------------------------------------------------------------- gear geometry
export function gearDims(p) {
  const R = 2.6 * p.partSize;
  const toothDepth = 0.42 * p.partSize;
  const root = R - toothDepth;
  const bore = 0.72 * p.partSize;
  return { R, root, bore, toothDepth, teeth: 14 };
}
export const gearOuterR = (p) => gearDims(p).R;

// 2D gear outline points (in XZ once mapped). Shared by the shape and the walls.
function outlinePoints(p) {
  const { R, root, toothDepth, teeth } = gearDims(p);
  const seg = teeth * 4;
  const pts = [];
  for (let i = 0; i < seg; i++) {
    const phase = i % 4;
    const a = (i / seg) * Math.PI * 2;
    let r;
    if (phase === 0) r = root;
    else if (phase === 1) r = root + toothDepth * 0.9;
    else if (phase === 2) r = R;
    else r = root + toothDepth * 0.4;
    pts.push([Math.cos(a) * r, Math.sin(a) * r]);
  }
  return pts;
}

function gearShape(p) {
  const pts = outlinePoints(p);
  const shape = new THREE.Shape();
  pts.forEach(([x, y], i) => (i === 0 ? shape.moveTo(x, y) : shape.lineTo(x, y)));
  shape.closePath();
  const hole = new THREE.Path();
  hole.absarc(0, 0, gearDims(p).bore, 0, Math.PI * 2, true);
  shape.holes.push(hole);
  return shape;
}

// stand an extrusion (extruded along +Z) upright on +Y with its base at y=0.
// +90° (not -90°) so the shape's Y axis maps to world +Z, matching the wall/infill
// geometry — otherwise the solid top shell is mirrored in Z and looks rotated.
function standUp(geo) {
  geo.rotateX(Math.PI / 2);
  geo.computeBoundingBox();
  geo.translate(0, -geo.boundingBox.min.y, 0);
  geo.computeVertexNormals();
  return geo;
}

// Flat gear cross-section (with bore hole) lying in the XZ plane — used for the
// SLA cure patch glowing on the vat floor.
export function gearCrossSectionGeometry(p) {
  const geo = new THREE.ShapeGeometry(gearShape(p), 24);
  geo.rotateX(-Math.PI / 2);
  return geo;
}

// One solid, fully-capped gear layer of thickness = layerHeight, base at y=0.
function solidSlabGeometry(p) {
  const geo = new THREE.ExtrudeGeometry(gearShape(p), {
    depth: p.layerHeight, bevelEnabled: false, curveSegments: 26,
  });
  return standUp(geo);
}

// Vertical wall skin following a closed 2D loop, from y=0 to y=h (double-sided).
function wallGeometry(loop, h, close = true) {
  const n = loop.length;
  const pos = [];
  const idx = [];
  const count = close ? n : n - 1;
  for (let i = 0; i < count; i++) {
    const a = loop[i], b = loop[(i + 1) % n];
    const base = pos.length / 3;
    pos.push(a[0], 0, a[1], b[0], 0, b[1], b[0], h, b[1], a[0], h, a[1]);
    idx.push(base, base + 1, base + 2, base, base + 2, base + 3);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}

// Infill crosshatch for one layer: parallel bars at `angle`, trimmed to a disk of
// radius `R0` and split around the centre bore so infill never crosses the hole.
function infillGeometry(p, angleDeg) {
  const { root, bore } = gearDims(p);
  const R0 = root - 0.05;
  const boreR = bore + 0.05; // keep infill clear of the centre hole
  const wall = 0.05 * p.partSize;
  const spacing = 0.34 * p.partSize;
  const th = angleDeg * Math.PI / 180;
  const dir = [Math.cos(th), Math.sin(th)];
  const nrm = [-Math.sin(th), Math.cos(th)];
  const bars = [];
  const addBar = (t0, t1, o) => {
    const len = t1 - t0;
    if (len < 0.1) return;
    const mid = (t0 + t1) / 2;
    const cx = o * nrm[0] + mid * dir[0];
    const cz = o * nrm[1] + mid * dir[1];
    const box = new THREE.BoxGeometry(len, p.layerHeight, wall);
    box.deleteAttribute('uv'); // match wall geometry (position + normal only) for merge
    box.rotateY(-th);
    box.translate(cx, p.layerHeight / 2, cz);
    bars.push(box);
  };
  for (let o = -R0; o <= R0; o += spacing) {
    if (Math.abs(o) >= R0) continue;
    const L = Math.sqrt(R0 * R0 - o * o);
    if (Math.abs(o) >= boreR) {
      addBar(-L, L, o); // clear of the bore: one full chord
    } else {
      const tb = Math.sqrt(boreR * boreR - o * o); // split around the hole
      addBar(-L, -tb, o);
      addBar(tb, L, o);
    }
  }
  if (!bars.length) { const b = new THREE.BoxGeometry(0.001, 0.001, 0.001); b.deleteAttribute('uv'); return b; }
  return mergeGeometries(bars, false);
}

// One FDM layer: perimeter walls (outer + bore) + infill, thickness layerHeight.
function fdmSlabGeometry(p, angleDeg) {
  const loop = outlinePoints(p);
  const boreLoop = [];
  const b = gearDims(p).bore;
  for (let i = 0; i < 40; i++) {
    const a = (i / 40) * Math.PI * 2;
    boreLoop.push([Math.cos(a) * b, Math.sin(a) * b]);
  }
  const parts = [
    wallGeometry(loop, p.layerHeight),
    wallGeometry(boreLoop, p.layerHeight),
    infillGeometry(p, angleDeg),
  ];
  return mergeGeometries(parts, false);
}

// ---------------------------------------------------------------- layered part
// Common base: a pool of layer meshes revealed discretely. Completed layers are
// solid; the current layer is the only one that changes within its time slice,
// revealed by an X-sweep at CONSTANT height (so layer thickness never "grows").
class LayeredPart {
  constructor() {
    this.group = new THREE.Group();
    this.max = 90;
    this.meshes = [];
    this.sweepPlane = new THREE.Plane(new THREE.Vector3(-1, 0, 0), 0); // keep x <= front
    this._key = '';
    this.sweep = { x: 0, yTop: 0, yBottom: 0 };
    this._buildMaterials();
    for (let i = 0; i < this.max; i++) {
      const m = new THREE.Mesh(new THREE.BufferGeometry(), this.solidMat);
      m.castShadow = true; m.receiveShadow = true; m.visible = false;
      this.group.add(m);
      this.meshes.push(m);
    }
  }

  setColors(part, accent) {
    this.solidMat.color.set(part);
    this.activeMat.color.set(part);
    this.activeMat.emissive.set(accent);
  }

  // opts: { mode:'up'|'down', glow:0..1, reveal:0..1 (override sweep progress) }
  update(state, params, opts = {}) {
    this.ensureGeometry(params);
    const lh = params.layerHeight;
    const mode = opts.mode || 'up';
    const Xr = gearOuterR(params) * 1.06;
    const li = state.layerIndex;
    const done = state.done;
    const rp = opts.reveal != null ? opts.reveal : state.layerProgress;
    const front = done ? Xr : THREE.MathUtils.lerp(-Xr, Xr, rp);
    this.sweepPlane.constant = front;
    this.activeMat.emissiveIntensity = 0.35 + (opts.glow ?? 0.8) * 1.5;

    // reset
    for (const m of this.meshes) m.visible = false;
    const count = state.layerCount;

    const place = (i, y, mat) => {
      const m = this.meshes[i];
      m.visible = true; m.material = mat; m.position.y = y;
      m.geometry = this.geomFor(i, count);
    };

    if (done || mode === 'up') {
      const top = done ? count : li;
      for (let j = 0; j < top && j < this.max; j++) place(j, j * lh, this.solidMat);
      if (!done && li < this.max) place(li, li * lh, this.activeMat);
      this.sweep = { x: front, yBottom: li * lh, yTop: (li + 1) * lh };
    } else {
      // 'down': part hangs; finished layers lifted by one layer, active forms at floor
      for (let j = 0; j < li && j < this.max; j++) place(j, lh + j * lh, this.solidMat);
      if (li < this.max) place(li, 0, this.activeMat);
      this.sweep = { x: front, yBottom: 0, yTop: lh };
    }
  }

  // which geometry a given layer index uses (overridden by subclasses)
  geomFor() { return this._sharedGeo; }

  dispose() {
    for (const m of this.meshes) m.geometry.dispose();
    this.solidMat.dispose(); this.activeMat.dispose();
  }
}

// Solid part (SLA / SLS / MJF): capped slabs, double-sided so no see-through.
export class SolidPart extends LayeredPart {
  _buildMaterials() {
    this.solidMat = new THREE.MeshStandardMaterial({ color: 0xd9dde3, roughness: 0.6, metalness: 0.05, side: THREE.DoubleSide });
    this.activeMat = new THREE.MeshStandardMaterial({
      color: 0xd9dde3, roughness: 0.45, metalness: 0.0, emissive: 0x000000, emissiveIntensity: 1,
      clippingPlanes: [this.sweepPlane], side: THREE.DoubleSide,
      polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -2, // win over coplanar finished-layer top
    });
  }
  ensureGeometry(params) {
    const key = `${params.partSize}|${params.layerHeight}`;
    if (key === this._key) return;
    this._key = key;
    if (this._sharedGeo) this._sharedGeo.dispose();
    this._sharedGeo = solidSlabGeometry(params);
  }
  geomFor() { return this._sharedGeo; }
}

// FDM part: perimeter walls + infill grid (angle alternates per layer). The top
// layer prints as a solid top shell (no infill), like a real FDM part.
export class FDMPart extends LayeredPart {
  constructor() { super(); this.shellLayers = 1; }
  _buildMaterials() {
    this.solidMat = new THREE.MeshStandardMaterial({ color: 0xe7523b, roughness: 0.55, metalness: 0.05, side: THREE.DoubleSide });
    this.activeMat = new THREE.MeshStandardMaterial({
      color: 0xe7523b, roughness: 0.4, metalness: 0.0, emissive: 0x000000, emissiveIntensity: 1,
      clippingPlanes: [this.sweepPlane], side: THREE.DoubleSide,
      polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -2,
    });
  }
  ensureGeometry(params) {
    const key = `${params.partSize}|${params.layerHeight}`;
    if (key === this._key) return;
    this._key = key;
    for (const g of [this._even, this._odd, this._solid]) if (g) g.dispose();
    this._even = fdmSlabGeometry(params, 45);
    this._odd = fdmSlabGeometry(params, -45);
    this._solid = solidSlabGeometry(params); // top shell
  }
  geomFor(i, count) {
    if (count > 1 && i >= count - this.shellLayers) return this._solid; // solid top shell
    return (i % 2 === 0) ? this._even : this._odd;
  }
}
