import * as THREE from 'three';
import { SolidPart, gearOuterR } from '../part.js';
import { buildState } from '../timeline.js';
import { disposeGroup } from '../util.js';

const bedR = (p) => gearOuterR(p) + 1.5;
const REVEAL_FRAC = 0.16;  // last fraction of the clip drains the powder to reveal the part
const RECOAT_FRAC = 0.45;  // each layer: recoater covers with powder first, then the fuse pass redraws

// Shared powder-bed rig for SLS and MJF. Each layer plays out as: the fusing pass
// reveals the fresh layer proud of the powder, then the recoater sweeps and the
// powder rises to bury that layer. Because every finished layer sits under opaque
// powder, the coplanar layer boundaries can't z-fight. At the end the powder
// drains away to reveal the finished gear.
export class PowderBedScene {
  constructor(cfg) { this.cfg = cfg; }

  buildBase(root, params) {
    this.root = root;
    this.part = new SolidPart();
    root.add(this.part.group);

    const R = bedR(params);
    this.platform = new THREE.Mesh(
      new THREE.BoxGeometry(R * 2, 0.3, R * 2),
      new THREE.MeshStandardMaterial({ color: 0x23272e, roughness: 0.85, metalness: 0.1 }),
    );
    this.platform.position.y = -0.15; this.platform.receiveShadow = true;
    root.add(this.platform);

    this.powderMat = new THREE.MeshStandardMaterial({ color: 0x9a958c, roughness: 1.0, metalness: 0 });
    this.powder = new THREE.Mesh(new THREE.BoxGeometry(R * 2, 1, R * 2), this.powderMat);
    this.powder.castShadow = true; this.powder.receiveShadow = true;
    root.add(this.powder);

    this.recoater = new THREE.Mesh(
      new THREE.BoxGeometry(0.32, 0.7, R * 2 + 0.3),
      new THREE.MeshStandardMaterial({ color: 0x2a2e35, roughness: 0.5, metalness: 0.6 }),
    );
    this.recoater.castShadow = true;
    root.add(this.recoater);
  }

  // shared powder-bed callouts; subclasses add the fusing-tool anchor(s)
  baseCallouts(state, p) {
    const R = bedR(p);
    return [
      { id: 'part', world: new THREE.Vector3(gearOuterR(p) * 0.7, state.totalH * 0.9 + 0.3, 0) },
      { id: 'powder', world: new THREE.Vector3(gearOuterR(p) * 0.2, Math.max(0.1, state.currentTopH * 0.4), R - 0.3) },
      { id: 'recoater', world: this.recoater.position.clone().add(new THREE.Vector3(0, 0.4, 0)) },
    ];
  }

  updateBase(t, state, params) {
    const p = params;
    const lh = p.layerHeight;
    const R = bedR(p);

    // compressed build timeline: finish building, then drain the powder
    const buildEnd = p.duration * (1 - REVEAL_FRAC);
    const tBuild = t <= buildEnd ? t * (p.duration / buildEnd) : p.duration;
    const bs = buildState(tBuild, p);
    const drain = THREE.MathUtils.clamp((t - buildEnd) / (p.duration - buildEnd), 0, 1);
    const drainE = drain * drain * (3 - 2 * drain);

    // Each layer: RECOAT first (recoater sweeps, fresh powder rises and covers
    // the previous fused surface), then FUSE (laser/lamp sweeps and redraws the
    // part on the new surface).
    const lp = bs.layerProgress;
    const recoating = !bs.done && lp < RECOAT_FRAC;
    const fusing = !bs.done && lp >= RECOAT_FRAC;
    const recoatP = THREE.MathUtils.clamp(lp / RECOAT_FRAC, 0, 1);
    const fuseP = THREE.MathUtils.clamp((lp - RECOAT_FRAC) / (1 - RECOAT_FRAC), 0, 1);

    // The active layer is only drawn during the fuse pass (covered by powder before).
    this.part.setColors(p.partColor, p.accentColor);
    this.part.update(bs, p, { mode: 'up', glow: 0.9, reveal: fusing ? fuseP : 0 });

    const powderTone = p.powderDarkness ?? p.agentDarkness ?? 0.55;
    this.powderMat.color.setHSL(0.09, 0.12, THREE.MathUtils.lerp(0.6, 0.24, powderTone));

    // Powder surface: sits just below the active layer's top so the fused
    // cross-section shows flush on the surface. During recoat it rises from the
    // previous surface to the new one, burying the last fused layer.
    const i = bs.layerIndex;
    const eps = 0.04 * lh;
    const fuseSurface = (i + 1) * lh - eps;
    let level;
    if (bs.done) level = THREE.MathUtils.lerp(bs.totalH, -0.28, drainE);
    else if (recoating) level = THREE.MathUtils.lerp(i * lh - eps, fuseSurface, recoatP);
    else level = fuseSurface;
    const lvl = Math.max(0.02, level);
    this.powder.geometry.dispose();
    this.powder.geometry = new THREE.BoxGeometry(R * 2, lvl, R * 2);
    this.powder.position.y = lvl / 2;
    this.powder.visible = p.showMachine && level > 0.03;
    this.platform.visible = p.showMachine;

    // recoater sweeps across during the recoat phase, spreading fresh powder
    const rx = THREE.MathUtils.lerp(-R - 0.4, R + 0.4, recoatP);
    this.recoater.position.set(rx, fuseSurface + 0.32, 0);
    this.recoater.visible = p.showMachine && recoating;

    const sw = this.part.sweep;
    return {
      bs, R, building: !bs.done, drain, fusing, recoating,
      fuseP, front: sw.x, surfaceY: sw.yTop,
    };
  }

  dispose() { disposeGroup(this.root); }
}

export { bedR };
