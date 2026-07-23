import * as THREE from 'three';
import { PowderBedScene, bedR } from './powderbed.js';

// MJF: during the fuse phase a full-width carriage jets fusing agent and a wide
// IR lamp fuses the layer — the part appears with the wipe. Then the recoater
// (handled by the base) sweeps and fresh powder covers the fused layer.
export class MJFScene extends PowderBedScene {
  constructor() { super({ label: 'MJF · Multi Jet Fusion', accent: '#25c2a0' }); }

  build(root, params) {
    this.buildBase(root, params);
    const R = bedR(params);

    this.carriage = new THREE.Mesh(
      new THREE.BoxGeometry(0.45, 0.7, R * 2 + 0.3),
      new THREE.MeshStandardMaterial({ color: 0x14161b, roughness: 0.4, metalness: 0.5 }),
    );
    this.carriage.castShadow = true;
    root.add(this.carriage);

    this.lamp = new THREE.Mesh(
      new THREE.BoxGeometry(0.5, 0.32, R * 2 + 0.3),
      new THREE.MeshStandardMaterial({ color: 0x3a2320, emissive: 0xff5a2a, emissiveIntensity: 2.0 }),
    );
    this.lamp.castShadow = true;
    root.add(this.lamp);
    this.lampGlow = new THREE.Mesh(
      new THREE.BoxGeometry(1.3, 0.05, R * 2 + 0.3),
      new THREE.MeshBasicMaterial({ color: 0xff5a2a, transparent: true, opacity: 0.3, depthWrite: false }),
    );
    root.add(this.lampGlow);
  }

  update(t, state, params) {
    const info = this.updateBase(t, state, params);
    const p = params;
    const y = info.surfaceY;
    const active = info.fusing;

    // carriage leads just ahead of the fusing front; lamp fuses at the front
    this.carriage.visible = active;
    this.carriage.position.set(info.front + 0.5, y + 0.5, 0);
    this.lamp.visible = active;
    this.lamp.position.set(info.front, y + 0.42, 0);
    this.lampGlow.visible = active;
    this.lampGlow.position.set(info.front, y + 0.05, 0);
    this.lamp.material.emissiveIntensity = 1.8;
  }

  getCallouts(state, p) {
    const base = this.baseCallouts(state, p).filter((c) => c.id === 'part' || c.id === 'powder');
    const top = state.done ? state.totalH + 1.2 : this.lamp.position.y;
    return [
      ...base,
      { id: 'lamp', world: new THREE.Vector3(0.2, top + 0.3, 0) },
      { id: 'carriage', world: new THREE.Vector3(-0.6, top + 0.5, 0) },
    ];
  }
}
