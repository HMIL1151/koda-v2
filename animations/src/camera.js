import * as THREE from 'three';

// Orthographic isometric camera rig with three modes:
//   static : fixed azimuth/elevation/zoom
//   orbit  : azimuth advances over time
//   push   : frustum tightens (dolly-in feel) over time
export class CameraRig {
  constructor(aspect) {
    this.cam = new THREE.OrthographicCamera(-1, 1, 1, -1, -50, 100);
    this.target = new THREE.Vector3(0, 1.1, 0);
    this.radius = 12;
    this.setAspect(aspect);
  }

  setAspect(aspect) {
    this.aspect = aspect;
  }

  // frustumHeight scales the visible world height; zoom shrinks it.
  update(t, p) {
    const frustumH = 6.4 / p.camZoom;
    let az = p.camAzimuth;
    let extraZoom = 1;

    if (p.camMode === 'orbit') {
      az += p.orbitSpeed * t;
    } else if (p.camMode === 'push') {
      const tn = p.duration > 0 ? Math.min(1, t / p.duration) : 0;
      extraZoom = 1 - p.pushAmount * (tn * tn * (3 - 2 * tn)); // ease
    }

    const h = (frustumH * extraZoom) / 2;
    const w = h * this.aspect;
    this.cam.left = -w; this.cam.right = w;
    this.cam.top = h; this.cam.bottom = -h;

    const azr = az * Math.PI / 180;
    const elr = p.camElevation * Math.PI / 180;
    const r = this.radius;
    this.cam.position.set(
      this.target.x + r * Math.cos(elr) * Math.sin(azr),
      this.target.y + r * Math.sin(elr),
      this.target.z + r * Math.cos(elr) * Math.cos(azr),
    );
    this.cam.up.set(0, 1, 0);
    this.cam.lookAt(this.target);
    this.cam.updateProjectionMatrix();
  }
}
