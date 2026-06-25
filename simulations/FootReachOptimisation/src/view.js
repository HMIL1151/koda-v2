// view.js — canvas ownership, world->screen transform, drawing primitives.
// World units are mm with +x right, +y DOWN (matches the kinematics: the foot
// hangs below the hips at positive y). Unlike the TorsoAngle view (fixed scale +
// x-pan camera), this one AUTO-FITS a supplied world bounding box into the canvas
// so the whole reachable workspace stays in frame as the geometry is optimised.

export const canvas = document.getElementById('legCanvas');
export const ctx = canvas.getContext('2d');

// Transform: screen = (world - origin) * scale + offset. Filled by fit().
const VIEW = { scale: 1, ox: 0, oy: 0, offX: 0, offY: 0 };

// Fit a world bounding box {minX,maxX,minY,maxY} into the canvas with `pad` px
// of margin, preserving aspect ratio (uniform scale, +y still points down).
export function fit(bounds, pad = 36) {
    const w = canvas.width - 2 * pad, h = canvas.height - 2 * pad;
    const bw = Math.max(1e-6, bounds.maxX - bounds.minX);
    const bh = Math.max(1e-6, bounds.maxY - bounds.minY);
    VIEW.scale = Math.min(w / bw, h / bh);
    // Centre the box in the canvas.
    VIEW.ox = (bounds.minX + bounds.maxX) / 2;
    VIEW.oy = (bounds.minY + bounds.maxY) / 2;
    VIEW.offX = canvas.width / 2;
    VIEW.offY = canvas.height / 2;
}

export function cv(p) {
    return { x: VIEW.offX + (p.x - VIEW.ox) * VIEW.scale,
             y: VIEW.offY + (p.y - VIEW.oy) * VIEW.scale };
}
export function cvLen(d) { return d * VIEW.scale; }

// Inverse of cv(): canvas pixel -> world mm, using the latest fit() transform.
export function toWorld(sx, sy) {
    return { x: VIEW.ox + (sx - VIEW.offX) / VIEW.scale,
             y: VIEW.oy + (sy - VIEW.offY) / VIEW.scale };
}

// Pointer event -> canvas pixel coords (handles any CSS scaling of the canvas).
export function eventToCanvas(ev) {
    const r = canvas.getBoundingClientRect();
    return { x: (ev.clientX - r.left) * canvas.width / r.width,
             y: (ev.clientY - r.top) * canvas.height / r.height };
}

export function clear() { ctx.clearRect(0, 0, canvas.width, canvas.height); }

export function line(a, b, color, width = 2, dash = []) {
    const A = cv(a), B = cv(b);
    ctx.save();
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineCap = 'round';
    ctx.setLineDash(dash);
    ctx.beginPath(); ctx.moveTo(A.x, A.y); ctx.lineTo(B.x, B.y); ctx.stroke();
    ctx.restore();
}

export function dot(p, r, color) {
    const P = cv(p);
    ctx.save(); ctx.fillStyle = color;
    ctx.beginPath(); ctx.arc(P.x, P.y, r, 0, Math.PI * 2); ctx.fill(); ctx.restore();
}

// Filled square marker in screen pixels (used for the workspace point cloud).
export function pixel(p, size, color) {
    const P = cv(p);
    ctx.fillStyle = color;
    ctx.fillRect(P.x - size / 2, P.y - size / 2, size, size);
}

// Axis-aligned rectangle given in WORLD coords {minX,maxX,minY,maxY}.
export function rect(r, { stroke = null, fill = null, width = 2, dash = [] } = {}) {
    const A = cv({ x: r.minX, y: r.minY }), B = cv({ x: r.maxX, y: r.maxY });
    ctx.save();
    if (fill)   { ctx.fillStyle = fill; ctx.fillRect(A.x, A.y, B.x - A.x, B.y - A.y); }
    if (stroke) {
        ctx.strokeStyle = stroke; ctx.lineWidth = width; ctx.setLineDash(dash);
        ctx.strokeRect(A.x, A.y, B.x - A.x, B.y - A.y);
    }
    ctx.restore();
}

// Text positioned in WORLD coords, drawn in screen pixels.
export function label(p, text, color, { align = 'center', baseline = 'middle', font = 'bold 11px monospace', dx = 0, dy = 0 } = {}) {
    const P = cv(p);
    ctx.save(); ctx.fillStyle = color; ctx.font = font;
    ctx.textAlign = align; ctx.textBaseline = baseline;
    ctx.fillText(text, P.x + dx, P.y + dy); ctx.restore();
}

// Text positioned directly in SCREEN pixels (overlays: titles, sparkline, etc.).
export function screenText(x, y, text, color, { align = 'left', baseline = 'top', font = '12px monospace' } = {}) {
    ctx.save(); ctx.fillStyle = color; ctx.font = font;
    ctx.textAlign = align; ctx.textBaseline = baseline;
    ctx.fillText(text, x, y); ctx.restore();
}

export function centeredText(text, color) {
    ctx.save();
    ctx.fillStyle = color; ctx.font = '13px monospace'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(text, canvas.width / 2, canvas.height / 2);
    ctx.restore();
}
