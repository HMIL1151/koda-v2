import * as THREE from 'three';
import { PowderBedScene } from './powderbed.js';

// SLS: a galvo-steered laser dot sinters powder along the layer during the fuse
// phase; the reveal front and the laser dot are one and the same. Then the
// recoater spreads fresh powder over it.
export class SLSScene extends PowderBedScene {
  constructor() { super({ label: 'SLS · Selective Laser Sintering', accent: '#ff3b6b' }); }

  build(root, params) {
    this.buildBase(root, params);
    this.dot = new THREE.Mesh(
      new THREE.SphereGeometry(0.12, 16, 16),
      new THREE.MeshStandardMaterial({ color: 0xff5a7a, emissive: 0xff3b6b, emissiveIntensity: 2.4 }),
    );
    root.add(this.dot);
    this.beam = new THREE.Mesh(
      new THREE.CylinderGeometry(0.015, 0.06, 3.4, 12, 1, true),
      new THREE.MeshBasicMaterial({ color: 0xff3b6b, transparent: true, opacity: 0.4, side: THREE.DoubleSide, depthWrite: false }),
    );
    root.add(this.beam);
  }

  update(t, state, params) {
    const info = this.updateBase(t, state, params);
    const p = params;
    const R = info.R;

    const z = Math.sin(info.bs.layerProgress * Math.PI * 8) * R * 0.55;
    const y = info.surfaceY + 0.02;
    const active = info.fusing;
    this.dot.visible = active; this.dot.position.set(info.front, y, z);
    this.beam.visible = active; this.beam.position.set(info.front, y + 1.7, z);

    this.dot.material.emissive.set(p.accentColor);
    this.dot.material.color.set(p.accentColor);
    this.beam.material.color.set(p.accentColor);
  }

  getCallouts(state, p) {
    const laser = state.done
      ? new THREE.Vector3(0, state.totalH + 1.4, 0)
      : this.dot.position.clone().add(new THREE.Vector3(0, 0.4, 0));
    return [...this.baseCallouts(state, p), { id: 'laser', world: laser }];
  }
}
