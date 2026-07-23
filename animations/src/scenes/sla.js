import * as THREE from 'three';
import { SolidPart, gearOuterR, gearCrossSectionGeometry } from '../part.js';
import { disposeGroup } from '../util.js';

const vatR = (p) => gearOuterR(p) + 1.3;

// Build a triangulated disk (rings × sectors) whose surface can be rippled each
// frame to read as a liquid. Base polar coords are stashed for the ripple.
function rippleDisk(R, rings, sectors) {
  const pos = [];
  const idx = [];
  const polar = [];
  polar.push([0, 0]); pos.push(0, 0, 0);
  for (let ri = 1; ri <= rings; ri++) {
    const r = (ri / rings) * R;
    for (let s = 0; s < sectors; s++) {
      const a = (s / sectors) * Math.PI * 2;
      polar.push([r, a]);
      pos.push(Math.cos(a) * r, 0, Math.sin(a) * r);
    }
  }
  const ringStart = (ri) => 1 + (ri - 1) * sectors;
  for (let s = 0; s < sectors; s++) {
    const a = ringStart(1) + s, b = ringStart(1) + (s + 1) % sectors;
    idx.push(0, a, b);
  }
  for (let ri = 1; ri < rings; ri++) {
    for (let s = 0; s < sectors; s++) {
      const a = ringStart(ri) + s, b = ringStart(ri) + (s + 1) % sectors;
      const c = ringStart(ri + 1) + s, d = ringStart(ri + 1) + (s + 1) % sectors;
      idx.push(a, c, b, b, c, d);
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.userData.polar = polar;
  g.computeVertexNormals();
  return g;
}

// SLA (bottom-up): the build plate rises and the part hangs below it, growing
// downward — each new layer cures at the bottom against the vat floor, UV from
// BELOW. The resin is a visible, rippling liquid the part is drawn up out of.
export class SLAScene {
  build(root, params) {
    this.root = root;
    this.part = new SolidPart();
    root.add(this.part.group);

    const R = vatR(params);

    // glass tank (simple translucency — no refraction, so the part stays crisp)
    const glass = new THREE.MeshStandardMaterial({
      color: 0x9fb4c4, roughness: 0.1, metalness: 0.1,
      transparent: true, opacity: 0.1, side: THREE.DoubleSide, depthWrite: false,
    });
    this.tank = new THREE.Mesh(new THREE.CylinderGeometry(R, R, 3.2, 56, 1, true), glass);
    this.tank.position.y = 3.2 / 2 - 0.02;
    this.tank.renderOrder = 5;
    root.add(this.tank);

    // liquid resin body: clean translucent tint (submerged part shows through
    // undistorted). Fluidity comes from the rippling glossy surface, not refraction.
    this.resinMat = new THREE.MeshStandardMaterial({
      color: 0x3ea6c8, roughness: 0.25, metalness: 0,
      transparent: true, opacity: 0.5, depthWrite: false,
    });
    this.resin = new THREE.Mesh(new THREE.CylinderGeometry(R - 0.06, R - 0.06, 1, 56), this.resinMat);
    this.resin.renderOrder = 6;
    root.add(this.resin);

    // rippling, glossy liquid surface
    this.surfMat = new THREE.MeshPhysicalMaterial({
      color: 0x7fd4ea, roughness: 0.14, metalness: 0,
      transparent: true, opacity: 0.5, clearcoat: 1, clearcoatRoughness: 0.08,
      side: THREE.DoubleSide, depthWrite: false,
    });
    this.surf = new THREE.Mesh(rippleDisk(R - 0.06, 12, 64), this.surfMat);
    this.surf.renderOrder = 7;
    root.add(this.surf);

    // build plate (rises); sized just under the part so the gear reads around it
    const pr = gearOuterR(params) * 0.82;
    this.plate = new THREE.Mesh(
      new THREE.CylinderGeometry(pr, pr, 0.16, 48),
      new THREE.MeshStandardMaterial({ color: 0x565d67, roughness: 0.45, metalness: 0.6 }),
    );
    this.plate.castShadow = true;
    root.add(this.plate);

    // UV projector under the transparent floor
    this.uvBox = new THREE.Mesh(
      new THREE.BoxGeometry(R * 1.4, 0.5, R * 1.4),
      new THREE.MeshStandardMaterial({ color: 0x1a1e26, roughness: 0.5, metalness: 0.4 }),
    );
    this.uvBox.position.y = -0.7;
    root.add(this.uvBox);
    // UV floods up through the transparent vat floor (masked SLA): a glowing pool
    // on the floor that extends past the part, so it reads clearly through the
    // translucent resin and around the part base — realistic and easy to see.
    this.curePool = new THREE.Mesh(
      new THREE.CircleGeometry(vatR(params) - 0.1, 56),
      new THREE.MeshBasicMaterial({ color: 0x9a7cff, transparent: true, opacity: 0.5, blending: THREE.AdditiveBlending, depthWrite: false }),
    );
    this.curePool.rotation.x = -Math.PI / 2;
    this.curePool.renderOrder = 7; // after the resin, so the UV glow adds on top of it
    root.add(this.curePool);
    // the fused layer's exact shape, glowing brighter on top of the pool
    this.curePatch = new THREE.Mesh(
      gearCrossSectionGeometry(params),
      new THREE.MeshBasicMaterial({ color: 0xd9ccff, transparent: true, opacity: 0.9, blending: THREE.AdditiveBlending, depthWrite: false }),
    );
    this.curePatch.renderOrder = 8;
    root.add(this.curePatch);
    // broad cone of UV light rising from the projector to the floor
    this.uvBeam = new THREE.Mesh(
      new THREE.CylinderGeometry(gearOuterR(params) * 0.5, gearOuterR(params) * 0.95, 0.62, 32, 1, true),
      new THREE.MeshBasicMaterial({ color: 0x7c5cff, transparent: true, opacity: 0.16, side: THREE.DoubleSide, blending: THREE.AdditiveBlending, depthWrite: false }),
    );
    root.add(this.uvBeam);
    this._sizeKey = '';
  }

  getCallouts(state, p) {
    const R = vatR(p);
    const builtTop = state.done ? state.totalH : p.layerHeight + state.layerIndex * p.layerHeight;
    return [
      { id: 'part', world: new THREE.Vector3(gearOuterR(p) * 0.85, builtTop * 0.55, 0) },
      { id: 'resin', world: new THREE.Vector3(-(R - 0.3), 0.35, 0.2) },
      { id: 'plate', world: new THREE.Vector3(0, builtTop + 0.2, 0) },
      { id: 'uv', world: new THREE.Vector3(0, -0.5, R * 0.4) },
    ];
  }

  rippleSurface(t, Hs) {
    const g = this.surf.geometry;
    const polar = g.userData.polar;
    const posAttr = g.getAttribute('position');
    for (let i = 0; i < polar.length; i++) {
      const [r, a] = polar[i];
      const y = 0.045 * Math.sin(6 * r - t * 2.6) + 0.03 * Math.sin(4 * (a + r) + t * 1.7);
      posAttr.setY(i, y);
    }
    posAttr.needsUpdate = true;
    g.computeVertexNormals();
    this.surf.position.y = Hs;
  }

  update(t, state, params) {
    const p = params;
    const lh = p.layerHeight;
    const R = vatR(p);

    this.part.setColors(p.partColor, p.accentColor);
    this.part.update(state, p, { mode: 'down', glow: 1.5 });

    const builtTop = state.done ? state.totalH : lh + state.layerIndex * lh;
    this.plate.position.y = builtTop + 0.09;
    this.plate.visible = p.showMachine;

    // shallow resin film the part is drawn up out of
    const Hs = 0.42;
    this.resin.geometry.dispose();
    this.resin.geometry = new THREE.CylinderGeometry(R - 0.06, R - 0.06, Hs, 56);
    this.resin.position.y = Hs / 2;
    this.resinMat.opacity = 0.32 + (p.resinLevel ?? 0.5) * 0.28;
    this.resinMat.color.set(p.partColor).lerp(new THREE.Color(0x59b6d6), 0.6);
    this.resin.visible = p.showMachine;
    this.rippleSurface(t, Hs);
    this.surf.visible = p.showMachine;
    this.tank.visible = p.showMachine;
    this.uvBox.visible = p.showMachine;

    // UV floods the layer from below (masked SLA), pulsing; the pool glows through
    // the resin and around the part, the patch marks the exact fused shape.
    const building = p.showMachine && !state.done;
    const pulse = 0.65 + 0.35 * Math.sin(t * 9);
    if (this._sizeKey !== `${p.partSize}`) { // rebuild patch on size change
      this._sizeKey = `${p.partSize}`;
      this.curePatch.geometry.dispose();
      this.curePatch.geometry = gearCrossSectionGeometry(p);
      this.curePool.geometry.dispose();
      this.curePool.geometry = new THREE.CircleGeometry(vatR(p) - 0.1, 56);
    }
    const uv = new THREE.Color(p.accentColor);
    this.curePool.visible = building; this.curePool.position.y = 0.06;
    this.curePool.material.color.copy(uv); this.curePool.material.opacity = 0.6 * pulse;
    this.curePatch.visible = building; this.curePatch.position.y = 0.03;
    this.curePatch.material.color.copy(uv).lerp(new THREE.Color(0xffffff), 0.4);
    this.curePatch.material.opacity = 0.85 * pulse;
    this.uvBeam.visible = building; this.uvBeam.position.y = -0.29;
    this.uvBeam.material.color.copy(uv); this.uvBeam.material.opacity = 0.16 * pulse;
  }

  dispose() { disposeGroup(this.root); }
}
