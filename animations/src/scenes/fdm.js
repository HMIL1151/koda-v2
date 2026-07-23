import * as THREE from 'three';
import { FDMPart, gearOuterR } from '../part.js';
import { disposeGroup, makeBuildPlate } from '../util.js';

export class FDMScene {
  build(root, params) {
    this.root = root;
    this.part = new FDMPart();
    root.add(this.part.group);

    this.plate = makeBuildPlate(gearOuterR(params) + 0.8);
    root.add(this.plate);

    // hotend / nozzle assembly
    this.nozzle = new THREE.Group();
    const dark = new THREE.MeshStandardMaterial({ color: 0x20242c, roughness: 0.5, metalness: 0.6 });
    const block = new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.55, 0.6), dark);
    block.position.y = 0.78; block.castShadow = true;
    const barrel = new THREE.Mesh(new THREE.CylinderGeometry(0.16, 0.16, 0.7, 20), dark);
    barrel.position.y = 1.3;
    this.heatMat = new THREE.MeshStandardMaterial({ color: 0x3a3f48, roughness: 0.4, metalness: 0.7, emissive: 0xff5a1e, emissiveIntensity: 0.6 });
    const heatBlock = new THREE.Mesh(new THREE.BoxGeometry(0.7, 0.3, 0.7), this.heatMat);
    heatBlock.position.y = 0.45;
    this.tipMat = new THREE.MeshStandardMaterial({ color: 0xd9a441, roughness: 0.3, metalness: 0.8, emissive: 0xff6a1e, emissiveIntensity: 1.0 });
    const tip = new THREE.Mesh(new THREE.ConeGeometry(0.18, 0.34, 20), this.tipMat);
    tip.position.y = 0.12; tip.rotation.x = Math.PI;
    this.nozzle.add(block, barrel, heatBlock, tip);
    root.add(this.nozzle);

    this.bead = new THREE.Mesh(new THREE.SphereGeometry(0.09, 16, 16),
      new THREE.MeshStandardMaterial({ color: 0xff7a3c, emissive: 0xff5a1e, emissiveIntensity: 1.6 }));
    root.add(this.bead);
  }

  getCallouts(state, p) {
    const R = gearOuterR(p);
    return [
      { id: 'nozzle', world: this.nozzle.position.clone().add(new THREE.Vector3(0, 0.7, 0)) },
      { id: 'part', world: new THREE.Vector3(R * 0.85, state.totalH * 0.6, R * 0.2) },
      { id: 'plate', world: new THREE.Vector3(-(R + 0.6), 0, 0.3) },
      // point at actual infill (between the bore and the rim), not the hole
      { id: 'infill', world: new THREE.Vector3(-R * 0.42, Math.max(0.2, state.currentTopH * 0.5), R * 0.42) },
    ];
  }

  update(t, state, params) {
    const p = params;
    this.part.setColors(p.partColor, p.accentColor);
    this.part.update(state, p, { mode: 'up', glow: p.nozzleTemp ?? 0.8 });

    this.plate.visible = p.showMachine;

    // nozzle rides the sweep front at the active layer height
    const sw = this.part.sweep;
    const R = gearOuterR(p);
    const z = Math.sin(state.layerProgress * Math.PI * 3) * R * 0.35;
    const active = p.showMachine && !state.done;
    this.nozzle.visible = active;
    this.nozzle.position.set(sw.x, sw.yTop + 0.02, z);
    this.bead.visible = active;
    this.bead.position.set(sw.x, sw.yBottom + 0.04, z);

    const glow = p.nozzleTemp ?? 0.8;
    this.heatMat.emissiveIntensity = 0.4 + glow * 1.1;
    this.heatMat.emissive.set(p.accentColor);
    this.tipMat.emissiveIntensity = 0.5 + glow;
    this.bead.material.emissive.set(p.accentColor);
    this.bead.material.color.set(p.accentColor);
  }

  dispose() { disposeGroup(this.root); }
}
