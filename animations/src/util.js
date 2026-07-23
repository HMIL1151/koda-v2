import * as THREE from 'three';

export function disposeGroup(root) {
  root.traverse((o) => {
    if (o.geometry) o.geometry.dispose();
    if (o.material) {
      const m = Array.isArray(o.material) ? o.material : [o.material];
      m.forEach((mm) => { if (mm.map) mm.map.dispose(); mm.dispose(); });
    }
  });
}

export function makeBuildPlate(radius, color = 0x2a2f38) {
  const g = new THREE.Mesh(
    new THREE.CylinderGeometry(radius, radius, 0.25, 64),
    new THREE.MeshStandardMaterial({ color, roughness: 0.85, metalness: 0.1 }),
  );
  g.position.y = -0.125;
  g.receiveShadow = true;
  return g;
}

export function sizeLabel(label) {
  const a = label.userData.aspect || 4;
  label.scale.set(0.62 * a, 0.62, 1);
}
