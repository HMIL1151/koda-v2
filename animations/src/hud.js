import * as THREE from 'three';

// A 2D overlay drawn on a canvas and composited into the WebGL output via a
// fullscreen quad, so titles / callouts / panels / frame guides are captured in
// the headless PNG export (canvas.toDataURL only sees the WebGL canvas).
export class HUD {
  constructor() {
    this.canvas = document.createElement('canvas');
    this.canvas.width = 16; this.canvas.height = 9;
    this.ctx = this.canvas.getContext('2d');
    this.tex = new THREE.CanvasTexture(this.canvas);
    this.tex.colorSpace = THREE.SRGBColorSpace;
    this.scene = new THREE.Scene();
    this.cam = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
    this.mat = new THREE.MeshBasicMaterial({ map: this.tex, transparent: true, depthTest: false, depthWrite: false });
    this.quad = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), this.mat);
    this.scene.add(this.quad);
    this.w = 16; this.h = 9;
  }

  setSize(w, h) {
    if (this.w === w && this.h === h) return;
    this.w = w; this.h = h;
    this.canvas.width = w; this.canvas.height = h;
  }

  // spec: { frameRect, camera, title, showFrame, callouts, calloutScale, materials, ratings, accent }
  render(renderer, spec) {
    this.draw(spec);
    this.tex.needsUpdate = true;
    renderer.setViewport(0, 0, this.w, this.h);
    renderer.setScissorTest(false);
    const prevAutoClear = renderer.autoClear;
    renderer.autoClear = false;
    renderer.clearDepth();
    renderer.render(this.scene, this.cam);
    renderer.autoClear = prevAutoClear;
  }

  // ---- 2D drawing ----
  draw(spec) {
    const { ctx } = this;
    const { frameRect: fr } = spec;
    ctx.clearRect(0, 0, this.w, this.h);
    // scale factor so overlay reads the same at any resolution (relative to 1080p)
    const S = fr.h / 1080;

    if (spec.showFrame) this.drawFrame(fr, S);
    if (spec.ratings && spec.ratings.show) this.drawRatings(fr, S, spec.ratings, spec.accent);
    if (spec.materials && spec.materials.show) this.drawMaterials(fr, S, spec.materials, spec.accent);
    if (spec.callouts && spec.callouts.length) this.drawCallouts(fr, S, spec);
    if (spec.title && spec.title.show) this.drawTitle(fr, S, spec.title, spec.accent);
  }

  drawFrame(fr, S) {
    const ctx = this.ctx;
    ctx.save();
    ctx.strokeStyle = 'rgba(255,255,255,0.5)';
    ctx.setLineDash([12 * S, 8 * S]);
    ctx.lineWidth = 2 * S;
    ctx.strokeRect(fr.x + 1, fr.y + 1, fr.w - 2, fr.h - 2);
    // thirds guides
    ctx.strokeStyle = 'rgba(255,255,255,0.12)';
    ctx.setLineDash([]);
    for (let i = 1; i < 3; i++) {
      const x = fr.x + fr.w * i / 3, y = fr.y + fr.h * i / 3;
      ctx.beginPath(); ctx.moveTo(x, fr.y); ctx.lineTo(x, fr.y + fr.h); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(fr.x, y); ctx.lineTo(fr.x + fr.w, y); ctx.stroke();
    }
    ctx.restore();
  }

  roundRect(x, y, w, h, r) {
    const ctx = this.ctx;
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }

  drawTitle(fr, S, title, accent) {
    const ctx = this.ctx;
    const scale = title.scale * S;
    const fs = 46 * scale;
    ctx.font = `700 ${fs}px ui-sans-serif, "Segoe UI", Roboto, sans-serif`;
    const padX = 26 * scale, padY = 15 * scale, dot = fs * 0.28, gap = fs * 0.42;
    const tw = ctx.measureText(title.text).width;
    const w = tw + padX * 2 + dot + gap;
    const h = fs + padY * 2;
    const cx = fr.x + fr.w * (0.5 + title.x * 0.5);
    const cy = fr.y + fr.h * (0.09 + title.y * 0.5);
    const x = cx - w / 2, y = cy - h / 2;
    ctx.save();
    ctx.fillStyle = 'rgba(14,18,25,0.72)';
    this.roundRect(x, y, w, h, h / 2); ctx.fill();
    ctx.fillStyle = accent;
    ctx.beginPath(); ctx.arc(x + padX + dot / 2, y + h / 2, dot, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#fff';
    ctx.textBaseline = 'middle';
    ctx.fillText(title.text, x + padX + dot + gap, y + h / 2 + fs * 0.02);
    ctx.restore();
  }

  drawCallouts(fr, S, spec) {
    const ctx = this.ctx;
    const scale = (spec.calloutScale || 1) * S;
    const fs = 26 * scale;
    ctx.font = `600 ${fs}px ui-sans-serif, "Segoe UI", Roboto, sans-serif`;
    const v = new THREE.Vector3();
    for (const c of spec.callouts) {
      let tx, ty, behind = false;
      if (c.mode === 'screen') {
        // fixed in the shot: leader points at a stationary screen position
        tx = fr.x + (0.5 + (c.sx ?? 0) * 0.5) * fr.w;
        ty = fr.y + (0.5 + (c.sy ?? 0) * 0.5) * fr.h;
      } else {
        // tracks a point on the part (moves as the camera orbits)
        v.copy(c.world).project(spec.camera);
        behind = v.z > 1;
        tx = fr.x + (v.x * 0.5 + 0.5) * fr.w;
        ty = fr.y + (-v.y * 0.5 + 0.5) * fr.h;
      }

      // static label box at its configured screen position (doesn't jump on orbit)
      const cx = fr.x + fr.w * (0.5 + (c.lx ?? 0) * 0.5);
      const cy = fr.y + fr.h * (0.5 + (c.ly ?? 0) * 0.5);
      const tw = ctx.measureText(c.text).width;
      const padX = 12 * scale, h = fs + 12 * scale;
      const w = tw + padX * 2;
      const bx = cx - w / 2, by = cy - h / 2;

      // leader from the box edge (side facing the target) to the target
      if (!behind) {
        const ex = THREE.MathUtils.clamp(tx, bx, bx + w);
        const ey = THREE.MathUtils.clamp(ty, by, by + h);
        ctx.strokeStyle = spec.accent;
        ctx.lineWidth = 2 * scale;
        ctx.beginPath(); ctx.moveTo(ex, ey); ctx.lineTo(tx, ty); ctx.stroke();
        ctx.fillStyle = spec.accent;
        ctx.beginPath(); ctx.arc(tx, ty, 5 * scale, 0, Math.PI * 2); ctx.fill();
      }

      ctx.fillStyle = 'rgba(14,18,25,0.85)';
      this.roundRect(bx, by, w, h, h / 2); ctx.fill();
      ctx.fillStyle = '#eef3f9';
      ctx.textBaseline = 'middle';
      ctx.fillText(c.text, bx + padX, by + h / 2 + 1);
    }
  }

  drawStar(cx, cy, r, filled, color) {
    const ctx = this.ctx;
    ctx.save();
    ctx.beginPath();
    for (let i = 0; i < 5; i++) {
      const a = -Math.PI / 2 + i * (Math.PI * 2 / 5);
      const a2 = a + Math.PI / 5;
      ctx.lineTo(cx + Math.cos(a) * r, cy + Math.sin(a) * r);
      ctx.lineTo(cx + Math.cos(a2) * r * 0.45, cy + Math.sin(a2) * r * 0.45);
    }
    ctx.closePath();
    if (filled >= 1) { ctx.fillStyle = color; ctx.fill(); }
    else if (filled <= 0) { ctx.strokeStyle = 'rgba(255,255,255,0.3)'; ctx.lineWidth = r * 0.16; ctx.stroke(); }
    else {
      ctx.strokeStyle = 'rgba(255,255,255,0.3)'; ctx.lineWidth = r * 0.16; ctx.stroke();
      ctx.clip(); ctx.fillStyle = color;
      ctx.fillRect(cx - r, cy - r, r * 2 * filled, r * 2);
    }
    ctx.restore();
  }

  drawStars(x, y, stars, S, color) {
    const r = 11 * S, gap = 5 * S;
    for (let i = 0; i < 5; i++) {
      const f = Math.max(0, Math.min(1, stars - i));
      this.drawStar(x + r + i * (r * 2 + gap), y, r, f, color);
    }
    return 5 * (r * 2 + gap);
  }

  drawRatings(fr, S, ratings, accent) {
    const ctx = this.ctx;
    const pad = 22 * S;
    const w = 340 * S;
    const titleFs = 24 * S, rowFs = 20 * S, rowH = 40 * S;
    const rows = ratings.rows;
    const h = pad * 2 + titleFs + 14 * S + rows.length * rowH;
    const x = fr.x + 26 * S, y = fr.y + fr.h * 0.5 - h / 2;
    ctx.save();
    ctx.fillStyle = 'rgba(14,18,25,0.78)';
    this.roundRect(x, y, w, h, 16 * S); ctx.fill();
    ctx.fillStyle = accent;
    ctx.font = `700 ${titleFs}px ui-sans-serif, "Segoe UI", Roboto, sans-serif`;
    ctx.textBaseline = 'top';
    ctx.fillText(ratings.title, x + pad, y + pad);
    ctx.font = `500 ${rowFs}px ui-sans-serif, "Segoe UI", Roboto, sans-serif`;
    let ry = y + pad + titleFs + 14 * S;
    for (const row of rows) {
      ctx.fillStyle = '#c7d0dc';
      ctx.textBaseline = 'middle';
      ctx.fillText(row.label, x + pad, ry + rowH / 2);
      this.drawStars(x + w - pad - 5 * (22 * S + 5 * S), ry + rowH / 2, row.stars, S, accent);
      ry += rowH;
    }
    ctx.restore();
  }

  drawMaterials(fr, S, materials, accent) {
    const ctx = this.ctx;
    const pad = 22 * S;
    const w = 300 * S;
    const titleFs = 24 * S, rowFs = 21 * S, rowH = 40 * S;
    const items = materials.items;
    const h = pad * 2 + titleFs + 14 * S + items.length * rowH;
    const x = fr.x + fr.w - w - 26 * S, y = fr.y + fr.h * 0.5 - h / 2;
    ctx.save();
    ctx.fillStyle = 'rgba(14,18,25,0.78)';
    this.roundRect(x, y, w, h, 16 * S); ctx.fill();
    ctx.fillStyle = accent;
    ctx.font = `700 ${titleFs}px ui-sans-serif, "Segoe UI", Roboto, sans-serif`;
    ctx.textBaseline = 'top';
    ctx.fillText(materials.title, x + pad, y + pad);
    ctx.font = `500 ${rowFs}px ui-sans-serif, "Segoe UI", Roboto, sans-serif`;
    let ry = y + pad + titleFs + 14 * S;
    for (const name of items) {
      ctx.fillStyle = accent;
      ctx.beginPath(); ctx.arc(x + pad + 5 * S, ry + rowH / 2, 5 * S, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = '#dbe3ec';
      ctx.textBaseline = 'middle';
      ctx.fillText(name, x + pad + 18 * S, ry + rowH / 2);
      ry += rowH;
    }
    ctx.restore();
  }
}

// Fit a target-aspect rectangle inside a canvas (letterbox), in canvas pixels.
export function fitRect(cw, ch, aspect) {
  let w = cw, h = cw / aspect;
  if (h > ch) { h = ch; w = ch * aspect; }
  return { x: (cw - w) / 2, y: (ch - h) / 2, w, h };
}
